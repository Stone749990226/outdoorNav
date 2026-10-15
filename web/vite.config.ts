import { defineConfig } from "vitest/config";
import react from "@vitejs/plugin-react";
export default defineConfig({
  plugins: [react()],
  server: {
    fs: {
      allow: [decodeURIComponent(new URL("..", import.meta.url).pathname)],
    },
    proxy: { "/api": "http://127.0.0.1:8765" },
  },
  test: { include: ["src/**/*.test.ts"] },
});
