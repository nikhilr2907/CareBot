# Docker setup

| Folder | Contents |
|---|---|
| `standard/` | Shared ROS image and Compose service, with no personal host paths |
| `user/` | Optional Compose override for WSL AI instructions/settings and reference assets |

## Standard environment

From the repository root:

```bash
docker compose -f docker/standard/docker-compose.yml up -d --build
```

VS Code's default Dev Container uses this shared configuration. The build context
and workspace bind mount resolve to the repository root.

## User environment

Enable Docker Desktop integration for your WSL distribution. From WSL, in the
repository root:

```bash
cp docker/user/.env.example docker/user/.env
# Edit docker/user/.env: replace YOUR_WSL_USER with your actual username.
docker compose --env-file docker/user/.env \
  -f docker/standard/docker-compose.yml \
  -f docker/user/docker-compose.yml config --quiet
docker compose --env-file docker/user/.env \
  -f docker/standard/docker-compose.yml \
  -f docker/user/docker-compose.yml up -d --build
```

The local `.env` is ignored by Git. Use existing absolute paths; the mounts refuse
to create missing host directories. Run the standard file first: Compose resolves
relative paths against its directory, including when applying the user override.

In VS Code select **Dev Containers: Attach to Running Container...**, choose the
`carerobot_dev` container, then open `/workspace`. The default **Reopen in Container**
configuration only selects the standard environment. To select both Compose files
there, add the user file to `dockerComposeFile` and export the variables from your
local `.env` into the environment that starts VS Code; listing the file alone does
not load `docker/user/.env` for interpolation.

## Mounted user files

| Host path variable | Container location | Purpose |
|---|---|---|
| `CODEX_HOST_DIR` | `/root/.codex` | Codex settings, instructions, skills and state |
| `CLAUDE_HOST_DIR` | `/root/.claude` | Claude instructions, skills, settings and state |
| `CLAUDE_HOST_CONFIG` | `/root/.claude.json` | Claude user configuration |
| `CARE_PROJECT_DIR` | `/care_reference` | Read-only previous project README and assets |
| `CARE_PROJECT_DIR` + `/configs` | `/care_configs` | Read-only ML environment configs |

After recreation, the supplied Windows logo path becomes
`/care_reference/assets/logo.png`. The previous README becomes
`/care_reference/README.md`.

AI mounts are writable, so container changes also change WSL state. They can
contain credentials; their contents must remain outside the repository and image.
Host-specific hooks, commands and keychain authentication may need adjustment.
These mounts do not install CLI tools or VS Code extensions. Repository-local
`AGENTS.md` and `CLAUDE.md` already remain available through `/workspace`.
