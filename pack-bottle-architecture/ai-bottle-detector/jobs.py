from enum import Enum
from dataclasses import dataclass


class JobStatus(str, Enum):
    CREATED = "CREATED"
    CAPTURING = "CAPTURING"
    PROCESSING = "PROCESSING"
    DONE = "DONE"
    FAILED = "FAILED"


@dataclass
class JobResult:
    raw_image: any = None
    yolo_processed_image: any = None
    processed_image: any = None
    bottles: list = None
    pick_and_place: dict = None


@dataclass
class Job:
    id: str
    status: JobStatus
    created_at: float
    result: JobResult | None = None
    error: str | None = None


JOBS: dict[str, Job] = {}
