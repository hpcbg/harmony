from fastapi.responses import StreamingResponse
from fastapi import FastAPI, BackgroundTasks, HTTPException
from fastapi.responses import Response
import uuid
import time
import cv2

from camera import Camera
from pipeline import process_frame
from jobs import Job, JobStatus, JobResult, JOBS
import utils.json_config

CONFIG = utils.json_config.load("config/detect_obb.json")

app = FastAPI(title="Bottle Perception API")

camera = Camera(CONFIG["RTSP_URL"])


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

        frame = camera.get_frame()
        if frame is None:
            raise RuntimeError("No camera frame available")

        job.status = JobStatus.PROCESSING
        result = process_frame(frame.copy())

        job.result = JobResult(
            raw_image=frame,
            yolo_processed_image=result["yolo_processed_image"],
            processed_image=result["processed_image"],
            bottles=result["bottles"],
            pick_and_place=result["pick_and_place"]
        )

        job.status = JobStatus.DONE

    except Exception as e:
        job.status = JobStatus.FAILED
        job.error = str(e)


@app.post("/api/v1/jobs")
def create_job(background_tasks: BackgroundTasks):
    job_id = str(uuid.uuid4())
    JOBS[job_id] = Job(job_id, JobStatus.CREATED, time.time())
    background_tasks.add_task(run_job, job_id)
    return {"job_id": job_id, "status": JobStatus.CREATED}


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


@app.get("/api/v1/jobs/{job_id}/image/yolo_processed")
def processed_image(job_id: str):
    job = JOBS.get(job_id)
    if not job or not job.result:
        raise HTTPException(404)

    _, img = cv2.imencode(".jpg", job.result.yolo_processed_image)
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


@app.get("/api/v1/jobs/{job_id}/pick-place")
def pick_place(job_id: str):
    job = JOBS.get(job_id)
    if not job or not job.result:
        raise HTTPException(404)
    return job.result.pick_and_place


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
