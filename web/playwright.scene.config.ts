import {defineConfig} from '@playwright/test';
export default defineConfig({
  testDir:'./tests',testMatch:'scene-outdoor.spec.ts',timeout:30000,workers:1,
  use:{baseURL:'http://127.0.0.1:18763',viewport:{width:1440,height:1080},locale:'zh-CN',trace:'retain-on-failure'},
  webServer:{command:'../.venv/bin/python ../robonix/system/scene/tests/outdoor_preview.py',url:'http://127.0.0.1:18763/outdoor',timeout:15000},
});
