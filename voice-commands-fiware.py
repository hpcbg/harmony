"""
Real-time microphone speech recognizer using Vosk + SoundDevice.
Detected keywords are published to a FIWARE Orion-LD Context Broker as
NGSI-LD entities.

MIT License — all dependencies (Vosk, SoundDevice, requests) are MIT-licensed.
Vosk models are Apache 2.0 licensed (compatible with MIT projects).

Architecture:
    Microphone → Vosk (speech recognition) → FIWARE Orion-LD (NGSI-LD)

Usage:
    python voice-commands-fiware.py                            # full transcription, English
    python voice-commands-fiware.py --keywords                 # built-in keyword spotting
    python voice-commands-fiware.py --keywords stop go cap     # custom keyword list
    python voice-commands-fiware.py --fiware                   # keywords + FIWARE publish
    python voice-commands-fiware.py --fiware --broker http://192.168.1.10:1026
    python voice-commands-fiware.py --lang de                  # German
    python voice-commands-fiware.py --list-mics                # show available microphones
    python voice-commands-fiware.py --mic 2                    # use microphone device index 2
    python voice-commands-fiware.py --model /path/to/vosk-model
"""

# MIT License
#
# Copyright (c) 2024
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.

import argparse
import json
import os
import queue
import sys
import urllib.request
import zipfile
from datetime import datetime, timezone
from pathlib import Path

import sounddevice as sd

try:
    import requests as _requests
    _HAS_REQUESTS = True
except ImportError:
    _HAS_REQUESTS = False

try:
    from vosk import KaldiRecognizer, Model
except ImportError:
    print("Vosk not found. Install it with:  pip install vosk")
    sys.exit(1)


# ---------------------------------------------------------------------------
# Model registry — small, fast, offline models (Apache 2.0 licensed)
# ---------------------------------------------------------------------------
MODELS = {
    "en": {
        "name": "vosk-model-small-en-us-0.15",
        "url":  "https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip",
        "size": "~40 MB",
    },
    "de": {
        "name": "vosk-model-small-de-0.15",
        "url":  "https://alphacephei.com/vosk/models/vosk-model-small-de-0.15.zip",
        "size": "~45 MB",
    },
    "fr": {
        "name": "vosk-model-small-fr-0.22",
        "url":  "https://alphacephei.com/vosk/models/vosk-model-small-fr-0.22.zip",
        "size": "~41 MB",
    },
    "es": {
        "name": "vosk-model-small-es-0.42",
        "url":  "https://alphacephei.com/vosk/models/vosk-model-small-es-0.42.zip",
        "size": "~39 MB",
    },
    "zh": {
        "name": "vosk-model-small-cn-0.22",
        "url":  "https://alphacephei.com/vosk/models/vosk-model-small-cn-0.22.zip",
        "size": "~42 MB",
    },
}

SAMPLE_RATE  = 16000   # Hz — Vosk models expect 16 kHz mono
BLOCK_SIZE   = 8000    # samples per audio callback (~0.5 s)
MODELS_DIR   = Path.home() / ".cache" / "vosk_models"

# Default command keywords — pass --keywords [WORD ...] to override
DEFAULT_KEYWORDS: list[str] = ["stop", "go", "cap", "give", "pick"]

# ANSI colours for keyword highlights (degrades gracefully on plain terminals)
_RESET  = "\033[0m"
_BOLD   = "\033[1m"
_COLORS = {
    "stop":     "\033[91m",   # bright red
    "go":       "\033[92m",   # bright green
    "cap":      "\033[93m",   # bright yellow
    "give": "\033[94m",   # bright blue
    "pick":     "\033[95m",   # bright magenta
}
_DEFAULT_COLOR = "\033[96m"   # bright cyan for any extra keywords

# ---------------------------------------------------------------------------
# FIWARE publisher (NGSIv2 API)
# ---------------------------------------------------------------------------
# We use the NGSIv2 API (/v2/) rather than NGSI-LD (/ngsi-ld/v1/) because
# fiware/orion-ld:1.6.0 requires the -experimental flag to enable NGSI-LD
# endpoints, which is not set in the standard Docker image entrypoint.
# NGSIv2 is always enabled, simpler, and fully sufficient for our use case.
# fiware_bridge.py subscribes via NGSIv2 as well.

DEFAULT_BROKER       = "http://localhost:1026"
DEFAULT_SERVICE      = "openiot"        # Fiware-Service header (logical tenant)
DEFAULT_SERVICEPATH  = "/"             # Fiware-Servicepath header
ENTITY_TYPE          = "VoiceCommand"
ENTITY_ID            = "VoiceCommand:operator-1"


class FiwarePublisher:
    """Publish voice command detections to a FIWARE Orion broker via NGSIv2.

    Uses Fiware-Service and Fiware-Servicepath headers to scope entities
    into a logical tenant — the same pattern used by IoT Agents and M5Stack
    devices:

        fiware_service     = "openiot"
        fiware_servicepath = "/"
        device_id          = "VoiceCommand:operator-1"

    On first call, creates the entity with POST /v2/entities.
    Subsequent calls update attributes with PATCH /v2/entities/<id>/attrs.
    """

    def __init__(
        self,
        broker_url:   str,
        source_id:    str = "mic-0",
        service:      str = DEFAULT_SERVICE,
        servicepath:  str = DEFAULT_SERVICEPATH,
        verbose:      bool = False,
    ):
        if not _HAS_REQUESTS:
            print("ERROR: 'requests' package is required for FIWARE publishing.")
            print("       Install it with:  pip install requests")
            sys.exit(1)

        self.broker      = broker_url.rstrip("/")
        self.source_id   = source_id
        self.service     = service
        self.servicepath = servicepath
        self.verbose     = verbose
        self._created    = False
        self._entity_url = f"{self.broker}/v2/entities/{ENTITY_ID}"
        self._attrs_url  = f"{self._entity_url}/attrs"
        self._headers    = {
            "Content-Type":       "application/json",
            "Accept":             "application/json",
            "Fiware-Service":     self.service,
            "Fiware-Servicepath": self.servicepath,
        }

    def _log(self, msg: str):
        if self.verbose:
            print(f"  [FIWARE] {msg}")

    def _ensure_entity(self):
        """Create the NGSIv2 entity if it doesn't already exist."""
        if self._created:
            return

        body = {
            "id":   ENTITY_ID,
            "type": ENTITY_TYPE,
            "command": {
                "type":  "Text",
                "value": "",
            },
            "confidence": {
                "type":  "Number",
                "value": 0.0,
            },
            "sourceId": {
                "type":  "Text",
                "value": self.source_id,
            },
            "timestamp": {
                "type":  "DateTime",
                "value": self._iso_now(),
            },
        }

        try:
            r = _requests.post(
                f"{self.broker}/v2/entities",
                headers=self._headers,
                json=body,
                timeout=5,
            )
            if r.status_code == 201:
                self._log(f"Entity created: {ENTITY_ID} "
                          f"(service={self.service}, path={self.servicepath})")
            elif r.status_code == 422:  # Unprocessable — already exists
                self._log("Entity already exists, will PATCH attributes.")
            else:
                print(f"  [FIWARE] WARNING: entity creation returned {r.status_code}: {r.text}")
                return
        except _requests.exceptions.ConnectionError:
            print(f"  [FIWARE] ERROR: cannot reach broker at {self.broker}")
            print("           Is Orion-LD running?  See docker-compose.yml")
            return

        self._created = True

    @staticmethod
    def _iso_now() -> str:
        return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"

    def publish(self, command: str, confidence: float = 1.0):
        """Update the entity attributes to trigger a subscription notification."""
        self._ensure_entity()
        if not self._created:
            return

        now  = self._iso_now()
        body = {
            "command":    {"type": "Text",     "value": command.upper()},
            "confidence": {"type": "Number",   "value": confidence},
            "timestamp":  {"type": "DateTime", "value": now},
        }

        try:
            r = _requests.patch(
                self._attrs_url,
                headers=self._headers,
                json=body,
                timeout=5,
            )
            if r.status_code == 204:
                self._log(f"Published '{command}' at {now}")
            else:
                print(f"  [FIWARE] WARNING: PATCH returned {r.status_code}: {r.text}")
        except _requests.exceptions.ConnectionError:
            print(f"  [FIWARE] ERROR: lost connection to broker at {self.broker}")

    def check_connection(self) -> bool:
        try:
            r = _requests.get(f"{self.broker}/v2/entities", timeout=3)
            return r.status_code < 500
        except _requests.exceptions.RequestException:
            return False



# ---------------------------------------------------------------------------
# Model management
# ---------------------------------------------------------------------------

def download_model(lang: str) -> Path:
    """Download and unzip a Vosk model if it isn't cached yet."""
    if lang not in MODELS:
        print(f"Unknown language '{lang}'. Available: {', '.join(MODELS)}")
        sys.exit(1)

    info       = MODELS[lang]
    model_dir  = MODELS_DIR / info["name"]
    MODELS_DIR.mkdir(parents=True, exist_ok=True)

    if model_dir.exists():
        print(f"Using cached model: {model_dir}")
        return model_dir

    zip_path = MODELS_DIR / f"{info['name']}.zip"
    print(f"Downloading {info['name']} ({info['size']}) …")

    def _progress(block_num, block_size, total_size):
        downloaded = block_num * block_size
        if total_size > 0:
            pct = min(downloaded / total_size * 100, 100)
            bar = "█" * int(pct // 2) + "░" * (50 - int(pct // 2))
            print(f"\r  [{bar}] {pct:5.1f}%", end="", flush=True)

    urllib.request.urlretrieve(info["url"], zip_path, reporthook=_progress)
    print()  # newline after progress bar

    print("Extracting …")
    with zipfile.ZipFile(zip_path, "r") as zf:
        zf.extractall(MODELS_DIR)
    zip_path.unlink()

    print(f"Model saved to {model_dir}\n")
    return model_dir


def load_model(model_path: str | None, lang: str) -> Model:
    """Return a Vosk Model, downloading it first if necessary."""
    if model_path:
        path = Path(model_path)
        if not path.exists():
            print(f"Model path not found: {path}")
            sys.exit(1)
    else:
        path = download_model(lang)

    print(f"Loading model …", end=" ", flush=True)
    model = Model(str(path))
    print("ready.\n")
    return model


# ---------------------------------------------------------------------------
# Audio helpers
# ---------------------------------------------------------------------------

def list_microphones():
    """Print available input devices and exit."""
    print("Available microphones:\n")
    devices = sd.query_devices()
    for i, dev in enumerate(devices):
        if dev["max_input_channels"] > 0:
            marker = " ◀ default" if i == sd.default.device[0] else ""
            print(f"  [{i:2d}] {dev['name']}{marker}")
    print()


# ---------------------------------------------------------------------------
# Keyword helpers
# ---------------------------------------------------------------------------

def _extract_keywords(text: str, keywords: set[str]) -> list[str]:
    """Return matched keywords found in text.

    Vosk grammar mode sometimes fuses tokens (e.g. 'stopopover' instead of
    'stop').  We match by containment so a keyword is detected as long as its
    exact string appears anywhere inside a token, then we return the clean
    keyword name — never the raw garbled token.
    """
    found: list[str] = []
    for token in text.lower().split():
        for kw in keywords:
            if kw in token and kw not in found:
                found.append(kw)
    return found


def _fmt_keyword(kw: str) -> str:
    """Return a coloured, bold, uppercased keyword label."""
    col = _COLORS.get(kw, _DEFAULT_COLOR)
    return f"{col}{_BOLD}{kw.upper()}{_RESET}"


# ---------------------------------------------------------------------------
# Recognition loop
# ---------------------------------------------------------------------------

def recognize(model: Model, device: int | None, keywords: list[str] | None,
              publisher: "FiwarePublisher | None" = None):
    """Stream audio from the microphone and print recognised text / keywords."""
    audio_queue: queue.Queue[bytes] = queue.Queue()

    kw_set: set[str] = set()
    if keywords is not None:
        kw_set = {k.lower() for k in keywords}
        # Tell Vosk to only score these words + silence filler.
        # The grammar is a JSON array; "[\"word1\", \"word2\", \"[unk]\"]"
        # The special token "[unk]" lets Vosk absorb out-of-vocabulary audio
        # gracefully instead of forcing a keyword match on every sound.
        grammar = json.dumps([w.lower() for w in keywords] + ["[unk]"])
        recognizer = KaldiRecognizer(model, SAMPLE_RATE, grammar)
    else:
        recognizer = KaldiRecognizer(model, SAMPLE_RATE)
        recognizer.SetWords(True)

    def _callback(indata, frames, time_info, status):
        if status:
            print(f"[audio warning] {status}", file=sys.stderr)
        audio_queue.put(bytes(indata))

    device_info = sd.query_devices(device, "input")
    print(f"Microphone : {device_info['name']}")
    print(f"Sample rate: {SAMPLE_RATE} Hz")

    if kw_set:
        kw_display = "  ".join(
            f"{_COLORS.get(k, _DEFAULT_COLOR)}{_BOLD}{k.upper()}{_RESET}"
            for k in sorted(kw_set)
        )
        print(f"Mode       : keyword spotting")
        print(f"Keywords   : {kw_display}")
    else:
        print(f"Mode       : full transcription")

    print("─" * 50)
    print("Listening … (press Ctrl+C to stop)\n")

    with sd.RawInputStream(
        samplerate=SAMPLE_RATE,
        blocksize=BLOCK_SIZE,
        device=device,
        dtype="int16",
        channels=1,
        callback=_callback,
    ):
        while True:
            data = audio_queue.get()

            if recognizer.AcceptWaveform(data):
                result = json.loads(recognizer.Result())
                text   = result.get("text", "").strip()
                if not text:
                    continue

                if kw_set:
                    # ── keyword mode ──
                    # Don't print raw Vosk tokens — they may be garbled fusions.
                    # Instead, print only the clean keyword names we matched.
                    found = _extract_keywords(text, kw_set)
                    if found:
                        print("\r" + " " * 60 + "\r", end="")  # clear partial line
                        labels = "  ".join(_fmt_keyword(kw) for kw in found)
                        print(f"▶ {labels}")
                        # Publish each detected keyword to FIWARE
                        if publisher:
                            for kw in found:
                                publisher.publish(kw)
                else:
                    # ── full transcription mode ──
                    print(f"▶ {text}")

            else:
                partial = json.loads(recognizer.PartialResult())
                partial_text = partial.get("partial", "").strip()
                if partial_text:
                    if kw_set:
                        # Show "listening…" instead of confusing raw tokens
                        display = "  ".join(
                            _fmt_keyword(kw)
                            for kw in _extract_keywords(partial_text, kw_set)
                        )
                        line = f"  … {display}" if display else "  … listening"
                    else:
                        line = f"  … {partial_text}"
                    print(line, end="\r", flush=True)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Real-time microphone speech recogniser (Vosk, offline, MIT-compatible)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python voice-commands-fiware.py                              # full transcription
  python voice-commands-fiware.py --keywords                   # spot built-in keywords
  python voice-commands-fiware.py --keywords stop go pick      # spot custom keywords
  python voice-commands-fiware.py --fiware                     # keywords + FIWARE publish
  python voice-commands-fiware.py --fiware --broker http://192.168.1.10:1026
  python voice-commands-fiware.py --lang fr                    # French transcription
  python voice-commands-fiware.py --list-mics                  # show microphones
  python voice-commands-fiware.py --mic 1                      # specific mic
  python voice-commands-fiware.py --model ~/my-vosk-model      # custom model path
        """,
    )
    p.add_argument("--lang",      default="en",  choices=MODELS.keys(),
                   help="Language code for automatic model download (default: en)")
    p.add_argument("--model",     default=None,  metavar="PATH",
                   help="Path to a local Vosk model directory (overrides --lang)")
    p.add_argument("--mic",       default=None,  type=int, metavar="INDEX",
                   help="Microphone device index (default: system default)")
    p.add_argument("--list-mics", action="store_true",
                   help="List available microphones and exit")
    p.add_argument(
        "--keywords",
        nargs="*",
        metavar="WORD",
        default=None,
        help=(
            "Enable keyword-spotting mode. "
            "Pass words to override the defaults "
            f"({', '.join(DEFAULT_KEYWORDS)}), "
            "or use --keywords alone to use the defaults."
        ),
    )
    # ── FIWARE flags ────────────────────────────────────────────────────────
    fiware = p.add_argument_group("FIWARE / NGSI-LD publishing")
    fiware.add_argument(
        "--fiware",
        action="store_true",
        help=(
            "Publish detected keywords to a FIWARE Orion-LD Context Broker "
            "(implies --keywords if no keywords are specified)."
        ),
    )
    fiware.add_argument(
        "--broker",
        default=DEFAULT_BROKER,
        metavar="URL",
        help=f"Orion broker base URL (default: {DEFAULT_BROKER})",
    )
    fiware.add_argument(
        "--service",
        default=DEFAULT_SERVICE,
        metavar="NAME",
        help=f"Fiware-Service header / logical tenant (default: {DEFAULT_SERVICE})",
    )
    fiware.add_argument(
        "--servicepath",
        default=DEFAULT_SERVICEPATH,
        metavar="PATH",
        help=f"Fiware-Servicepath header (default: {DEFAULT_SERVICEPATH})",
    )
    fiware.add_argument(
        "--source-id",
        default="mic-0",
        metavar="ID",
        help="Identifier of this microphone / operator (stored as sourceId on the entity)",
    )
    fiware.add_argument(
        "--fiware-verbose",
        action="store_true",
        help="Print each NGSI-LD request/response status",
    )
    return p


def main():
    args = build_parser().parse_args()

    if args.list_mics:
        list_microphones()
        return

    # ── resolve keyword list ────────────────────────────────────────────────
    # --fiware alone implies --keywords (with defaults)
    if args.fiware and args.keywords is None:
        args.keywords = []

    if args.keywords is not None:
        keywords = args.keywords if args.keywords else DEFAULT_KEYWORDS
    else:
        keywords = None

    # ── set up FIWARE publisher ─────────────────────────────────────────────
    publisher: FiwarePublisher | None = None
    if args.fiware:
        if keywords is None:
            print("ERROR: --fiware requires keyword mode. Add --keywords.")
            sys.exit(1)
        publisher = FiwarePublisher(
            broker_url=args.broker,
            source_id=args.source_id,
            service=args.service,
            servicepath=args.servicepath,
            verbose=args.fiware_verbose,
        )
        print(f"FIWARE broker     : {args.broker}")
        print(f"Fiware-Service    : {args.service}")
        print(f"Fiware-Servicepath: {args.servicepath}")
        if publisher.check_connection():
            print("Broker status : ✔ reachable\n")
        else:
            print("Broker status : ✘ NOT reachable — detections will be skipped\n"
                  "                (start Orion-LD with:  docker compose up -d)\n")

    model = load_model(args.model, args.lang)

    try:
        recognize(model, args.mic, keywords, publisher)
    except KeyboardInterrupt:
        print("\n\nStopped.")
    except sd.PortAudioError as exc:
        print(f"\nAudio error: {exc}")
        print("Try --list-mics to see available devices, then pick one with --mic INDEX")
        sys.exit(1)


if __name__ == "__main__":
    main()