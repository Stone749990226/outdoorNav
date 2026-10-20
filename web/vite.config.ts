import { defineConfig } from "vitest/config";
import react from "@vitejs/plugin-react";
import { loadEnv } from "vite";
export default defineConfig(({ mode }) => ({
  plugins: [react()],
  server: {
    fs: {
      allow: [decodeURIComponent(new URL("..", import.meta.url).pathname)],
    },
    proxy: {
      "/api":
        loadEnv(mode, ".", "OUTDOORNAV_").OUTDOORNAV_API_URL ||
        "http://127.0.0.1:8765",
    },
  },
  test: { include: ["src/**/*.test.ts"] },
}));
