# Episode video gallery

A dependency-free Python server for the `view_iphone.mp4` in each `episode_*`
directory of a single recording session. It serves an allowlist of those videos,
generated thumbnail images, and gallery assets. Other recording data is not served.
Byte-range requests allow native browser playback and seeking without downloading
the entire recording first. Optional `ffprobe` and `ffmpeg` provide durations and
thumbnail images; originals are read without being modified.

From the repository root:

```sh
python3 scripts/episode_gallery/server.py data/episodes/rec_20261104-204103
```

Open <http://127.0.0.1:8765>. To expose it using a temporary Cloudflare URL:

```sh
cloudflared tunnel --no-autoupdate --url http://127.0.0.1:8765 --protocol http2
```

Anyone with the generated URL can view the videos. The machine and both processes
must remain running. The URL changes when the tunnel is recreated.

The current deployment uses user services named `episode-video-gallery.service`
and `episode-video-tunnel.service`:

```sh
systemctl --user status episode-video-gallery episode-video-tunnel
journalctl --user -u episode-video-tunnel -n 40 --no-pager
systemctl --user stop episode-video-gallery
```

Stopping the gallery also stops its tunnel. Start both again with:

```sh
systemctl --user start episode-video-gallery episode-video-tunnel
```

Services are started for this deployment, not enabled at login. Thumbnails are
cached under `run/episode-gallery/<session>/`. Restart the gallery to rescan a
session after adding episodes.
