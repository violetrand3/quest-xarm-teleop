const videos = [...document.querySelectorAll('video')];

for (const video of videos) {
  video.addEventListener('play', () => {
    for (const other of videos) {
      if (other !== video) other.pause();
    }
  });
  const showError = () => {
    video.closest('article').querySelector('.video-error').hidden = false;
  };
  video.addEventListener('error', showError);
  video.querySelector('source').addEventListener('error', showError);
  video.addEventListener('loadeddata', () => {
    video.closest('article').querySelector('.video-error').hidden = true;
  });
}
