# `context/` — knowledge base for your AI assistant

You will use an AI assistant (ChatGPT, Claude, Gemini, Copilot, Cursor,
Windsurf, a local model — whatever you normally use) while writing robot
scripts. Out of the box it knows Universal Robots and `ur_rtde` in general,
but **not this course's framework**, so it will happily invent methods that
do not exist, use degrees where the code expects radians, or write raw
URScript. This folder fixes that: give it to your assistant as context and
it will work with the same API, units and rules you have.

## What is in here

| file | give it to the AI when… | size |
|---|---|---|
| `INSTRUCTIONS.md` | **always** — the rules: how to help, what not to do (system prompt / custom instructions) | 8 KB |
| `KNOWLEDGE.md` | **always** — the facts: environment, the `ur5e` API with signatures, workspace, safety profiles, lab robot, typical errors | 16 KB |
| `CART_360.md` | **always, once you work on the real arm or around the cart** — the lab safety setup `cart_360`: what is checked, how to route moves around the cart, what each rejection means | 8 KB |
| `framework_source.md` | the AI needs to see the actual implementation (it is `src/ur5e/*.py` + the templates concatenated; regenerate with `./context/make_source_snapshot.sh` after a `git pull`) | ~110 KB |
| `ur_rtde/api_reference.txt` | you go below the framework (`robot.conn.control…`) and need exact `ur_rtde` call signatures | 95 KB |
| `ur_rtde/guides.txt`, `examples.txt`, `faq.txt`, `rtde_protocol_guide.txt`, `installation.txt`, `introduction.txt` | background on RTDE, realtime loops, servo/speed commands, the record/replay example | 150 KB total |
| `manual/UR5e_User_Manual_en_Global.txt` | pendant menus, I/O wiring and voltages, tool flange connector, safety settings, freedrive, installation variables — the official UR5e user manual (PolyScope 5.x) as searchable text | 720 KB |
| `manual/UR5e_User_Manual_en_Global.pdf` | the same manual with figures, for you (and for assistants that read PDFs) | 17 MB |

Start with the first three. Add the rest when a question needs it — most
tools have a context limit, and the manual alone is ~200k tokens.

## How to feed it to your tool

**Chat tools with "projects" / "knowledge" (ChatGPT Projects, Claude
Projects, Gemini Gems, Perplexity Spaces, Mistral Le Chat agents…)**
Create one project for the course. Paste `INSTRUCTIONS.md` into the
project's custom instructions / system prompt. Upload `KNOWLEDGE.md`,
`CART_360.md`, `framework_source.md` and `ur_rtde/api_reference.txt` as knowledge files;
upload the manual `.txt` (or `.pdf`) too if the tool allows the size. Every
new chat in that project then starts informed. When you ask for a script,
also paste the template you are starting from and your current
`workspace.json` (so it knows the box size).

**Editor assistants reading the repo (Claude Code, Codex, Cursor, Windsurf,
GitHub Copilot in VS Code, Gemini CLI, Aider…)**
Nothing to do — the repo root contains `AGENTS.md`, `CLAUDE.md` and
`.github/copilot-instructions.md`, which these tools pick up automatically
and which point here. Open the repo folder (not just your script) so the
tool can read `context/` and `src/ur5e/` itself. For Cursor you can
additionally add `context/INSTRUCTIONS.md` under *Settings → Rules*.

**Plain chat without files (free tiers, local models)**
Paste `INSTRUCTIONS.md` + `KNOWLEDGE.md` + `CART_360.md` as the first message of the
conversation (~9k tokens together), then your template and your question.

## Keep your own notes in the loop

The AI only knows what it is given. When you teach a new workspace, mount a
tool, or change the TCP, tell it — paste `ws.describe()` output or the
`workspace.json`. When it suggests a method that is not in `KNOWLEDGE.md §4`,
it is inventing; say "only use methods from KNOWLEDGE.md" and it will stop.

## Sources

- `INSTRUCTIONS.md`, `KNOWLEDGE.md`, `CART_360.md`, `framework_source.md`: written for
  this course from the framework code (status lines in the code mark what is
  validated for student use).
- `ur_rtde/`: text conversion of the ur_rtde 1.6.3 documentation,
  https://sdurobotics.gitlab.io/ur_rtde/ (SDU Robotics, MIT licence).
- `manual/`: Universal Robots, *UR5e User Manual*, e-Series, software 5.x,
  original English, © Universal Robots A/S — the official manual as
  downloadable from https://www.universal-robots.com/download/. Included for
  study use in this course.
