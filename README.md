# eYRC 2026-27: PacBot

Launch files and boilerplate for every PacBot task. Clone this once, and pull
when a new task is released.

Full instructions, rules and marking live on the task site:
**<https://themes.e-yantra.org/Pacbot_2627/>**

---

## Before you start

Install these once. Everything else the simulators need is bundled with them.

```sh
sudo apt update
sudo apt install -y libgl1 libx11-6 libxcb1 libxau6 libxdmcp6 \
                    mosquitto mosquitto-clients
pip3 install paho-mqtt
```

| What | Why |
|---|---|
| `libgl1` | OpenGL, for the simulator window. It belongs to your graphics driver, so it cannot be shipped here. **The one most likely to be missing.** |
| `libx11-6` `libxcb1` `libxau6` `libxdmcp6` | X11, for the window. Already there on a normal desktop; listed for minimal installs. |
| `mosquitto` `mosquitto-clients` | The MQTT broker your code and the simulator talk through. |
| `paho-mqtt` | The Python MQTT client. |

Installing something you already have does nothing, so run the whole command.

Then make sure the broker is up:

```sh
systemctl status mosquitto        # should say active (running)
sudo systemctl enable --now mosquitto
```

> Use `pip3` for paho-mqtt, not `apt`. The apt package is version 1.5.1; pip
> gives 2.x, which is what the tasks are tested against.

---

## What is here

```
task0/    task0_eval
task1a/   task_1a_launch          task_1a_boilerplate.py
task1b/   task_1b_launch   lib/   meshes/   task_1b_boilerplate.py
task2a/   task_2a_launch   lib/   meshes/   task_2a_boilerplate.py
task2b/   task_2b_launch   lib/   meshes/   task_2b_boilerplate.py   maze_2b.py
```

| | |
|---|---|
| `*_launch` | The simulator and evaluator for that task. Nothing to compile. |
| `*_boilerplate.py` | Your starting point. The MQTT plumbing is done; the thinking is yours. |
| `lib/` | Shared libraries the simulator needs. Do not move or delete them. |
| `meshes/` | Robot geometry, loaded at startup. |
| `maze_2b.py` | Task 2B only: the maze parser, with the official maze built in. |

**Keep each task folder together.** The launcher finds `lib/` and `meshes/`
next to itself, so you can move or rename a folder, but not split it up.


---

## Getting updates

New tasks are added to this repository as they are released. You will have your
own code in here, so set it aside before pulling:

```sh
git stash        # set your changes aside
git pull         # bring down the new task folders
git stash pop    # put your changes back
```

Do not forget the `git stash pop`. Until you run it your changes are shelved
and will not appear in your files. If the pop reports a conflict, open the file
it names and keep the lines you want.

---

## If something will not run

| Symptom | Cause |
|---|---|
| `error while loading shared libraries` | Something was moved out of `lib/`, or `libgl1` is missing. See **Before you start**. |
| `Connection refused` | The broker is not running. `sudo systemctl enable --now mosquitto` |
| Window opens, robot never moves | The simulator waits for your code. Start the boilerplate in a second terminal. |
| `Permission denied` | `chmod +x` the launcher. |
| `ModuleNotFoundError: paho` | `pip3 install paho-mqtt` |

---

Everything else, including the rules and the marking scheme for each task, is
on the task site: **<https://themes.e-yantra.org/Pacbot_2627/>**
