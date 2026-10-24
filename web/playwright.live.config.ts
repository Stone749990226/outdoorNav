import { defineConfig } from "@playwright/test";

export default defineConfig({
  testDir: "./tests",
  testMatch: "task-flow-live.spec.ts",
  timeout: 300000,
  workers: 1,
  reporter: "line",
  use: {
    baseURL: "http://127.0.0.1:19865",
    viewport: { width: 1920, height: 1080 },
    trace: "retain-on-failure",
    launchOptions: process.env.PLAYWRIGHT_EXECUTABLE_PATH
      ? { executablePath: process.env.PLAYWRIGHT_EXECUTABLE_PATH }
      : undefined,
  },
});
