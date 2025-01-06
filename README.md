# 2025-offseason
This is FRC-5026's robot code for the 2025 onseason ("REEFSCAPE").

## Set-up
- Clone the repo: ``git clone git@github.com:Iron-Panthers/2025-onseason``
- Install JDK-17
    - Windows: [https://www.oracle.com/java/technologies/downloads/](https://www.oracle.com/java/technologies/downloads/)
    - Linux: package manager of choice (pacman, apt, etc.)
- Use editor of choice

## Build and Deploy
(run all in root directory)
- Build project: ``./gradlew build``
- Deploy to robot: ``./gradlew deploy``
- Run formatter: ``./gradlew spotlessApply``
- Run simulation: ``./gradlew simulateJava``

## Extra Tools
Check for hardware/OS requirements.
Testing and debug tools:
- FRC Driver Station (NI) [https://www.ni.com/en/support/downloads/drivers/download.frc-game-tools.html#553883](https://www.ni.com/en/support/downloads/drivers/download.frc-game-tools.html#553883)
- AdvantageScope (Mechanical Advantage) [https://github.com/Mechanical-Advantage/AdvantageScope/releases](https://github.com/Mechanical-Advantage/AdvantageScope/releases)
- TDB Elastic (FRC-353) [https://github.com/Gold872/elastic-dashboard/releases](https://github.com/Gold872/elastic-dashboard/releases)
- Phoenix Tuner X (CTRE) [https://v6.docs.ctr-electronics.com/en/latest/docs/tuner/index.html](https://v6.docs.ctr-electronics.com/en/latest/docs/tuner/index.html)
    - only available on "app stores", use at own risk
