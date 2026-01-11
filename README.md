# Quickstart — PeakFTC2025

This repository contains the PeakFTC2025 robot code. The sections below explain how to set up the code locally, build it, and deploy it to the Peak FTC robot.

## 1. Create the local branch, build, and run on the robot

1. Clone the Quickstart repository (includes Pedro Pathing):
   https://github.com/Pedro-Pathing/Quickstart

2. Place or clone the `PeakFTC2025` code under:
   `TeamCode/src/main/java/org/firstinspires/ftc/teamcode`

3. Open the project in Android Studio and select the `Quickstart` folder as the base project.

4. Build the project and deploy it to the robot for testing.

**Note:** If a `pedropathing` folder exists in the ``TeamCode/src/main/java/org/firstinspires/ftc/teamcode`` repository (https://github.com/Pedro-Pathing/Quickstart), delete it to avoid duplicate files that may cause build errors. All Pedro Pathing–related changes are available under the `PeakFTC2025` folder.

## 2. Modify code and upload changes to the branch

- After creating your local branch, update existing source files or add new ones inside the `PeakFTC2025` folder.
- Build and test by deploying to the robot.

When changes are ready, add, commit, and push:

```bash
git add "path/to/file"
git commit -m "Your commit message describing the change"
git push
```

Replace `path/to/file` with the file(s) you changed. Use a clear commit message to describe the change.

## 3. Tips

- Verify the project builds in Android Studio before deploying.
- Keep commits small and focused to make reviews easier.
- Run tests on the robot after each significant change.

