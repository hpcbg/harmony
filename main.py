from fastapi.responses import StreamingResponse
from fastapi import FastAPI, BackgroundTasks, HTTPException, Request
from fastapi.responses import Response
import uuid
import time
import cv2
import logging

import fiware
from camera import Camera
from pipeline import process_frame
from jobs import Job, JobStatus, JobResult, JOBS
import utils.json_config

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)


CONFIG = utils.json_config.load("config/config.json")

app = FastAPI(title="Bottle Perception API")

camera = Camera(CONFIG["CAMERA"], CONFIG["SET_RESOLUTION"],
                CONFIG["WIDTH"], CONFIG["HEIGHT"])

fiware.ensure_entity()
fiware.ensure_command_subscription(CONFIG["APP_API_HOST"])


def mjpeg_generator():
    while True:
        frame = camera.get_frame()
        if frame is None:
            continue

        _, jpeg = cv2.imencode(".jpg", frame)

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" +
            jpeg.tobytes() +
            b"\r\n"
        )


def run_job(job_id: str):
    job = JOBS[job_id]

    try:
        job.status = JobStatus.CAPTURING
        fiware.update_job_status(job_id, "CAPTURING")  # ← NEW

        frame = camera.get_frame()
        if frame is None:
            raise RuntimeError("No camera frame available")

        job.status = JobStatus.PROCESSING
        fiware.update_job_status(job_id, "PROCESSING")  # ← NEW
        result = process_frame(frame.copy())

        job.result = JobResult(
            raw_image=frame,
            ai_processed_image=result["ai_processed_image"],
            processed_image=result["processed_image"],
            bottles=result["bottles"],
            pick_pose=result["pick_pose"]
        )

        job.status = JobStatus.DONE
        fiware.update_job_status(  # ← NEW
            job_id, "DONE",
            bottle_count=len(result["bottles"]),
            pick_pose=result["pick_pose"]
        )

    except Exception as e:
        job.status = JobStatus.FAILED
        job.error = str(e)
        fiware.update_job_status(job_id, "FAILED", error=str(e))  # ← NEW


@app.post("/api/v1/jobs")
def create_job(background_tasks: BackgroundTasks):
    job_id = str(uuid.uuid4())
    JOBS[job_id] = Job(job_id, JobStatus.CREATED, time.time())
    fiware.update_job_status(job_id, "CREATED")  # no more create_job_entity
    background_tasks.add_task(run_job, job_id)
    return {"job_id": job_id, "status": JobStatus.CREATED}


@app.post("/api/v1/fiware/notify")
async def fiware_notify(request: Request, background_tasks: BackgroundTasks):
    body = await request.json()
    print(body)
    for entity in body.get("data", []):
        command = entity.get("command", {}).get("value", "")
        if command == "START":
            result = create_job(background_tasks)
            logger.info("FIWARE triggered new job: %s", result["job_id"])
            fiware.reset_command()

    return {"ok": True}


@app.get("/api/v1/jobs/{job_id}/status")
def job_status(job_id: str):
    job = JOBS.get(job_id)
    if not job:
        raise HTTPException(404)
    return {"status": job.status}


@app.get("/api/v1/jobs/{job_id}/image/raw")
def raw_image(job_id: str):
    job = JOBS.get(job_id)
    if not job or not job.result:
        raise HTTPException(404)

    _, img = cv2.imencode(".jpg", job.result.raw_image)
    return Response(img.tobytes(), media_type="image/jpeg")


@app.get("/api/v1/jobs/{job_id}/image/ai-processed")
def processed_image(job_id: str):
    job = JOBS.get(job_id)
    if not job or not job.result:
        raise HTTPException(404)

    _, img = cv2.imencode(".jpg", job.result.ai_processed_image)
    return Response(img.tobytes(), media_type="image/jpeg")


@app.get("/api/v1/jobs/{job_id}/image/processed")
def processed_image(job_id: str):
    job = JOBS.get(job_id)
    if not job or not job.result:
        raise HTTPException(404)

    _, img = cv2.imencode(".jpg", job.result.processed_image)
    return Response(img.tobytes(), media_type="image/jpeg")


@app.get("/api/v1/jobs/{job_id}/objects")
def objects(job_id: str):
    job = JOBS.get(job_id)
    if not job or not job.result:
        raise HTTPException(404)
    return job.result.bottles


@app.get("/api/v1/jobs/{job_id}/pick-pose")
def pick_pose(job_id: str):
    job = JOBS.get(job_id)
    if not job or not job.result:
        raise HTTPException(404)
    return job.result.pick_pose


@app.get("/api/v1/camera/snapshot")
def camera_snapshot():
    frame = camera.get_frame()
    if frame is None:
        raise HTTPException(503, "Camera not ready")

    _, img = cv2.imencode(".jpg", frame)
    return Response(img.tobytes(), media_type="image/jpeg")


@app.get(
    "/api/v1/camera/live",
    summary="Live MJPEG stream from camera",
    description="""
⚠️ **IMPORTANT**

This endpoint provides a **continuous MJPEG video stream** from the camera.

❌ **Swagger UI CANNOT display this stream**  
✅ Open it directly in a browser, VLC, or HMI client

### How to use:
- Browser: `http://<host>:22001/api/v1/camera/live`
- VLC: *Media → Open Network Stream*
- SCADA / HMI video widget

This endpoint is **read-only** and does **not perform any image processing**.
""",
    responses={
        200: {
            "description": "MJPEG live stream (not compatible with Swagger UI)",
            "content": {
                "multipart/x-mixed-replace": {}
            }
        }
    }
)
def camera_live():
    return StreamingResponse(
        mjpeg_generator(),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )


@app.get("/health")
def health():
    return {
        "camera": camera.get_frame() is not None,
        "jobs": len(JOBS)
    }
