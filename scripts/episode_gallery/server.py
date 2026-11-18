#!/usr/bin/env python3
"""Serve only the iPhone videos from one recording session (no dependencies)."""

import argparse
from dataclasses import dataclass
from functools import partial
import html
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
import subprocess
import tempfile
from urllib.parse import urlsplit


STATIC = Path(__file__).resolve().parent


@dataclass
class Episode:
    name: str
    video: Path
    poster: Path | None
    duration: float | None


def discover(session: Path, cache: Path) -> dict[str, Episode]:
    cache.mkdir(parents=True, exist_ok=True)
    episodes = {}
    for folder in sorted(session.iterdir()):
        if not re.fullmatch(r"episode_\d+", folder.name):
            continue
        video = (folder / "view_iphone.mp4").resolve()
        if not video.is_file() or not video.is_relative_to(session):
            continue
        duration = None
        try:
            result = subprocess.run(
                ["ffprobe", "-v", "error", "-show_entries", "format=duration",
                 "-of", "json", str(video)],
                capture_output=True, text=True, check=True, timeout=15,
            )
            duration = float(json.loads(result.stdout)["format"]["duration"])
        except (OSError, subprocess.SubprocessError, ValueError, KeyError):
            pass

        stat = video.stat()
        poster = cache / f"{folder.name}-{stat.st_size}-{stat.st_mtime_ns}.jpg"
        if not poster.is_file():
            with tempfile.NamedTemporaryFile(suffix=".jpg", dir=cache) as temp:
                try:
                    subprocess.run(
                        ["ffmpeg", "-nostdin", "-loglevel", "error", "-y",
                         "-ss", "1", "-i", str(video), "-frames:v", "1",
                         "-vf", "scale=640:-2", "-q:v", "4", "-threads", "1",
                         temp.name],
                        check=True, capture_output=True, timeout=30,
                    )
                    poster.write_bytes(Path(temp.name).read_bytes())
                except (OSError, subprocess.SubprocessError):
                    pass
        episodes[folder.name] = Episode(
            folder.name, video, poster if poster.is_file() else None, duration,
        )
    return episodes


def render(session: Path, episodes: dict[str, Episode]) -> bytes:
    cards = []
    for episode in episodes.values():
        number = int(episode.name.removeprefix("episode_"))
        source = f"/video/{episode.name}.mp4"
        poster = f' poster="/poster/{episode.name}.jpg"' if episode.poster else ""
        duration = ""
        if episode.duration is not None:
            seconds = round(episode.duration)
            duration = f"{seconds // 60}:{seconds % 60:02d} · "
        size = episode.video.stat().st_size / 1_000_000
        cards.append(f'''<article id="{episode.name}">
          <video controls playsinline preload="none"{poster}
                 aria-label="Episode {number:02d}" width="640" height="480">
            <source src="{source}" type="video/mp4">
            Your browser cannot play this video. <a href="{source}">Open the MP4</a>.
          </video>
          <div class="episode-info">
            <div><h2>Episode {number:02d}</h2>
              <p>{duration}{size:.1f} MB</p></div>
            <a href="{source}" aria-label="Open Episode {number:02d} video">Open video <span aria-hidden="true">↗</span></a>
          </div>
          <p class="video-error" role="status" hidden>Video could not load. Try the Open video link or refresh the page.</p>
        </article>''')
    if not cards:
        cards.append('<p class="empty">No view_iphone.mp4 videos were found in this session.</p>')
    page = (STATIC / "index.html").read_text()
    for key, value in {
        "SESSION": html.escape(session.name),
        "COUNT": str(len(episodes)),
        "CARDS": "\n".join(cards),
    }.items():
        page = page.replace("{{" + key + "}}", value)
    return page.encode()


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "EpisodeGallery"

    def __init__(self, *args, session, episodes, **kwargs):
        self.session = session
        self.episodes = episodes
        super().__init__(*args, **kwargs)

    def do_HEAD(self):
        self.route(head=True)

    def do_GET(self):
        self.route(head=False)

    def end_headers(self):
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("Content-Security-Policy", "default-src 'none'; style-src 'self'; script-src 'self'; img-src 'self'; media-src 'self'; base-uri 'none'; frame-ancestors 'none'")
        super().end_headers()

    def route(self, head):
        try:
            path = urlsplit(self.path).path
            if path in ("/", "/index.html"):
                body = render(self.session, self.episodes)
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                if not head:
                    self.wfile.write(body)
                return
            static = {
                "/style.css": (STATIC / "style.css", "text/css; charset=utf-8"),
                "/gallery.js": (STATIC / "gallery.js", "text/javascript; charset=utf-8"),
            }
            if path in static:
                self.send_file(*static[path], head=head)
                return
            match = re.fullmatch(r"/(video|poster)/(episode_\d+)\.(mp4|jpg)", path)
            if match:
                kind, name, extension = match.groups()
                episode = self.episodes.get(name)
                if episode and kind == "video" and extension == "mp4":
                    self.send_file(episode.video, "video/mp4", head=head)
                    return
                if episode and kind == "poster" and extension == "jpg" and episode.poster:
                    self.send_file(episode.poster, "image/jpeg", head=head)
                    return
            self.send_error(404, "Not found")
        except (BrokenPipeError, ConnectionResetError):
            pass  # Browsers cancel video requests when pausing or seeking.
        except FileNotFoundError:
            self.send_error(404, "File no longer available")

    def send_file(self, path, content_type, head):
        with path.open("rb") as stream:
            stat = path.stat()
            size = stat.st_size
            etag = f'"{stat.st_mtime_ns:x}-{size:x}"'
            range_header = self.headers.get("Range") if not head else None
            if self.headers.get("If-Range") not in (None, etag, self.date_time_string(stat.st_mtime)):
                range_header = None
            if not range_header and self.headers.get("If-None-Match") == etag:
                self.send_response(304)
                self.send_header("ETag", etag)
                self.end_headers()
                return
            start, end = 0, size - 1
            if range_header:
                match = re.fullmatch(r"bytes=(\d*)-(\d*)", range_header)
                valid = bool(match and any(match.groups()))
                if valid:
                    first, last = match.groups()
                    try:
                        if first:
                            start = int(first)
                            end = min(int(last), size - 1) if last else size - 1
                        else:
                            suffix = int(last)
                            start, end = max(0, size - suffix), size - 1
                        valid = 0 <= start <= end < size
                    except ValueError:
                        valid = False
                if not valid:
                    self.send_response(416)
                    self.send_header("Content-Range", f"bytes */{size}")
                    self.send_header("Content-Length", "0")
                    self.end_headers()
                    return
            length = max(0, end - start + 1)
            self.send_response(206 if range_header else 200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(length))
            self.send_header("Accept-Ranges", "bytes")
            self.send_header("ETag", etag)
            self.send_header("Last-Modified", self.date_time_string(stat.st_mtime))
            self.send_header("Cache-Control", "public, max-age=3600")
            if range_header:
                self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
            self.end_headers()
            if head:
                return
            stream.seek(start)
            while length > 0:
                chunk = stream.read(min(256 * 1024, length))
                if not chunk:
                    break
                self.wfile.write(chunk)
                length -= len(chunk)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path, help="Recording folder containing episode_* directories")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=8765, type=int)
    parser.add_argument("--cache-dir", type=Path)
    args = parser.parse_args()
    session = args.session.expanduser().resolve()
    if not session.is_dir():
        parser.error(f"Session directory does not exist: {session}")
    cache = args.cache_dir or STATIC.parents[1] / "run" / "episode-gallery" / session.name
    episodes = discover(session, cache)
    handler = partial(Handler, session=session, episodes=episodes)
    server = ThreadingHTTPServer((args.host, args.port), handler)
    print(f"Serving {len(episodes)} episodes at http://{args.host}:{args.port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
