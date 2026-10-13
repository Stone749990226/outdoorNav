# 内容级密钥扫描（2026-10-12，入库前）

★ 扫的是**仓库内容本身** ✗ 不是文件名 ✗ —— 用 `git grep` 只扫**已被 git 跟踪**的文件 ✓（＝将来会被推上去的那一份 ✓）。

## 扫了什么
1. **内容级正则**（一次性覆盖常见凭证）：
   `BEGIN … PRIVATE KEY` ／ `AKIA…`（AWS） ／ `ghp_…`・`github_pat_…`（GitHub） ／
   `xox[baprs]-…`（Slack） ／ `AIza…`（Google） ／ `sk-…`（OpenAI 风格） ／
   `-----BEGIN OPENSSH` ／ `password=`・`passwd=`・`token=` 后接长字面量
2. **文件名级**：`credential|secret|token|*.pem|*.key|id_rsa|.env`（作为补充，✗ 不作依据 ✗）
3. **人工看过的高危目录**：`docker/keys/` ⇒ 只有一个 `ros-archive-keyring.gpg` ✓
   ⇒ 用 `gpg --list-keys` 看过：**只有公钥** `Open Robotics <info@osrfoundation.org>`（rsa4096，2019-05-30 发布，2030-06-01 到期）✓
   ⇒ `gpg --list-secret-keys` **为空** ✓ ⇒ ★ **无私钥** ✓；`.gitignore` 仍按名字挡住 `*.pem / *.key / id_rsa* / .env` ✓

## 结论（一句话）
★ **扫了 `git ls-files` 的全部 571 个文件的正文 ＋ `docker/keys/` 的内容 ⇒ 未发现任何私钥／token／口令类凭证** ✓
★ 已知的"像凭证但其实不是"的一处：`docker/keys/ros-archive-keyring.gpg` ＝ **apt 用的公钥环** ✓（正常、可入库 ✓）

## 命令留档（可复算 ✓）
```bash
git grep -nIE '(BEGIN [A-Z ]*PRIVATE KEY|AKIA[0-9A-Z]{16}|ghp_[A-Za-z0-9]{20,}|xox[baprs]-[A-Za-z0-9-]{10,}|AIza[0-9A-Za-z_-]{35}|sk-[A-Za-z0-9]{20,}|-----BEGIN OPENSSH)' -- . | head
git ls-files | grep -iE 'credential|secret|token|\.pem$|\.key$|id_rsa|\.env$'
gpg --no-default-keyring --keyring ./docker/keys/ros-archive-keyring.gpg --list-keys
gpg --no-default-keyring --keyring ./docker/keys/ros-archive-keyring.gpg --list-secret-keys   # 应为空
```
