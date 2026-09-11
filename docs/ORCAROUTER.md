# OrcaRouter provider

[OrcaRouter](https://www.orcarouter.ai) is an OpenAI-compatible AI gateway for
both models and agents. This repository ships it as a first-class provider for
the youBot demos, with **two independent ways to authenticate** that both end up
holding the same kind of credential.

| Choice | Provider id | How you get the key |
| --- | --- | --- |
| OrcaRouter - API | `orcarouter` | You paste an existing `sk-orca-…` key. No browser needed. |
| OrcaRouter - Auth | `orcarouter-oauth` | You sign in with your OrcaRouter account (OAuth 2.0 + PKCE) and a key is issued to you. |

## Origins

Authentication and inference are deliberately different origins:

| Purpose | Origin |
| --- | --- |
| Consent screen, code exchange | `https://www.orcarouter.ai` |
| Inference, model catalog | `https://api.orcarouter.ai/v1` |

The two are never derived from one another. A self-hosted deployment that runs
one shared origin sets `ORCA_BASE_URL`; separate deployments set
`ORCA_AUTH_BASE_URL` and `ORCA_API_BASE_URL`, which take precedence.

```bash
export ORCA_BASE_URL=https://orca.internal           # shared self-hosted origin
export ORCA_AUTH_BASE_URL=https://auth.internal      # explicit override (wins)
export ORCA_API_BASE_URL=https://api.internal/v1     # explicit override (wins)
```

Non-loopback origins must be HTTPS; plain HTTP is accepted only for
`localhost` / `127.0.0.1` / `[::1]` during local development.

## API key

```bash
export ORCAROUTER_API_KEY=sk-orca-…      # existing secret mechanism, wins over disk
python -m orcarouter.cli set-key         # or store it interactively
```

Keys are stored in the project credential file
(`$XDG_CONFIG_HOME/v-rep-youbot-demo/orcarouter.json`, mode `0600`). The
credential is never logged, never placed in a URL, and never sent to the
browser — the settings UI only ever receives a masked form.

## Connect with OrcaRouter (OAuth 2.0 + PKCE)

```bash
python -m orcarouter.cli connect          # Flow A: loopback redirect (default)
python -m orcarouter.cli connect --flow B # Flow B: out-of-band code
```

**Flow A (loopback redirect) is the default** because this client is a local
application — the settings UI and the CLI run on the same machine as the
simulation and can bind `127.0.0.1`, so the authorization code returns
automatically and the user clicks once. Flow B is available for deployments
that cannot receive a redirect.

Properties the implementation guarantees:

* a fresh verifier and `state` per attempt, from `secrets.token_bytes`;
* `code_challenge = base64url(sha256(verifier))` with no padding, and
  `code_challenge_method=S256` always — never `plain`, because the user can
  choose "show me a code" even on Flow A;
* the verifier never leaves the process until the exchange: not in a URL, a log
  line, an error message, or telemetry;
* `state` is compared with `hmac.compare_digest` **before** the code is used;
* no client secret and no pre-registered redirect URI;
* the granted `scope` from the exchange response is read back and checked, not
  assumed.

### The key is durable, not refreshable

OrcaRouter returns a long-lived API key — not an access/refresh token pair.
There is no refresh endpoint. So:

* the stored key is reused across restarts until OrcaRouter revokes it;
* a `401` from the relay is **terminal**: the exact account and credential
  generation that made the rejected request is marked `needsReauth`, and no
  fake refresh is attempted;
* the old secret is not deleted before a replacement succeeds;
* there is a cap of 10 PKCE-issued keys per user per 24 hours, so the client
  never re-authorizes on every launch.

Revoke access at <https://www.orcarouter.ai/console/authorized-apps>.

## Model catalog

The model list comes from `GET https://api.orcarouter.ai/v1/models` using your
key, so the selector shows what your workspace can actually call. Model ids keep
their `vendor/model` namespace. Each surface gets its own filtered view:

| Surface | Filter |
| --- | --- |
| Text chat | `?capability=chat` + a text-capable endpoint type |
| Image / audio / video understanding | chat **and** the matching `architecture.input_modalities` entry (fail closed) |
| Embeddings | `?capability=embedding` |
| Image generation | `?capability=image` |
| Video generation | `?capability=video` |
| Rerank | `?capability=rerank` |

Capabilities are never guessed from a model name. Live discovery is
authoritative; if it fails, a small verified fallback catalog is used and the UI
says so. The fallback is never merged into a successful live result.

## Using it from the demo

```bash
python 7_Demo_youBotPickAndPlace/code/orcarouter_agent.py --list-models
python 7_Demo_youBotPickAndPlace/code/orcarouter_agent.py --model orcarouter/auto \
    --prompt "Plan how to pick up the can at point A."
python -m orcarouter.web.app     # the settings UI
```

## Tests

```bash
python -m pytest tests/            # unit + integration, no network
python tests/e2e_ui.py             # live UI evidence (needs ORCAROUTER_API_KEY)
```

## Provider evidence

| Item | Source |
| --- | --- |
| Inference endpoint (OpenAI-compatible) | `https://api.orcarouter.ai/v1/chat/completions` |
| Model list endpoint | `GET https://api.orcarouter.ai/v1/models` (accepts a Bearer key) |
| Authorization / exchange | `https://www.orcarouter.ai/auth`, `POST https://www.orcarouter.ai/api/v1/auth/keys` |
| Discovery document | `https://www.orcarouter.ai/.well-known/openid-configuration` |
| Revocation / account management | `https://www.orcarouter.ai/console/authorized-apps` |
| Provider | <https://www.orcarouter.ai> |
| Discord | <https://discord.gg/YEubt8enRA> |
| X | <https://x.com/OrcaRouter> |
| Verification date | 2026-09-11 |

**Affiliation disclosure:** this integration is made by an engineer on the
OrcaRouter team.
