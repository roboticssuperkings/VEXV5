# VEX V5 Competition Code

This repository contains our **VEX V5 competition robot code** built with **PROS** and **LimLib**. The code handles driver control, autonomous routines, intake and scoring mechanisms, pistons, GPS, IMU, rotation tracking, and distance sensor alignment.

## Main Features

- **PROS + LemLib drivetrain control**
- **Odometry** using:
  - IMU
  - vertical tracking wheel
- **Autonomous routines** for:
  - states auto
  - skills
  - right side routines
  - solo AWP-style routines
- **Driver control** with arcade drive
- **Scoring systems** for:
  - intake
  - outtake
  - low goal scoring
  - high goal scoring
  - middle goal descore
- **Pneumatics** for scraper, hood/wing, and middle-goal mechanisms
- **Sensor-based movement** using distance sensors and GPS

## Hardware Configuration

### Drive Motors
- Left drive: ports `-12, -1, 11`
- Right drive: ports `10, 19, -20`

### Other Motors
- Low intake: port `13`
- Top intake: port `6`
- Outtake: port `9`

### Sensors
- GPS: port `17`
- IMU: port `7`
- Vertical rotation sensor: port `2`
- Distance sensor: port `5`
- Front distance sensor: port `14`

### Pneumatics
- Scraper piston: `'G'`
- Hood / descore wing piston: `'H'`
- Middle goal descore piston: `'C'`
- Middle goal piston: `'B'`

## Important Functions

### Utility / Mechanism Functions
- `intakeBlocks()` – runs intake inward
- `stopIntakeBlocks()` – stops intake and outtake
- `goalHighScoring()` – runs motors for high goal scoring
- `midgoalscoring()` – runs motors for middle goal actions
- `lowgoalscoring()` – runs motors for low goal scoring
- `hoodup()` / `hooddown()` – controls hood piston
- `scraperDown()` / `scraperUp()` – controls scraper piston

### Autonomous Routines
- `states_auto()`
- `skills_full_auton_75()`
- `skillsfirstgoal()`
- `rightlong()`
- `rightlongtomiddle()`
- `middletoleftlong()`
- `sevenballright()`
- `fourballrightrush()`
- `rightSideDoubleGoal_NEW()`
- `solopp()`
- `ruiguansoloopp()`

## Driver Control

In `opcontrol()`:
- Left joystick Y = forward/backward
- Right joystick X = turning
- Arcade drive is used through LemLib

### Controller Mapping
- `L2` = low intake one direction
- `DOWN` = low intake reverse
- `L1` = top intake
- `R1` = piston1 on
- `R2` = piston1 off
- `X` = scraper piston on
- `B` = scraper piston off
- `UP` = piston3 on
- `DOWN` = piston3 off
- `RIGHT` = piston2 on
- `LEFT` = piston2 off

## Notes

- The chassis uses **LemLib controller settings** for both linear and angular motion.
- Position is printed to the brain screen during initialization for debugging.
- Some autonomous routines are still experimental or tuning-based.
- Distance-sensor alignment is used in parts of autonomous for more accurate positioning.

## Build / Upload

This project is made for **PROS**.

Typical workflow:

```bash
git add .
git commit -m "describe changes"
git push
