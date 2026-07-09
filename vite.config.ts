import { defineConfig } from 'vite';

// GitHub Pages serves project sites from /<repo>/, so the built asset
// paths must be rooted there instead of "/". Local dev/preview keep "/".
export default defineConfig({
  base: process.env.GITHUB_PAGES ? '/robohand/' : '/',
});
