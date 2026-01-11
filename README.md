1. How to create this branch in the local PC, build the code and run on Peak FTC robot.
    1. Clone or Download FTC robotics source with Pedro pathing from this git hub repo. https://github.com/Pedro-Pathing/Quickstart
    2. Go into the folder Teamcode and Clone PeakFTC2025 (your branch or master) repo under this folder "TeamCode/src/main/java/org/firstinspires/ftc/teamcode"
    3. Open project in Android studio and select the folder "Quickstart" the base folder.
    4. Build the code and download to robot for testing
  
2.How to modify and upload the changes into the branch.
    After creating the branch in a local folder.
    Update the code => Create new source file or update the existing source code in to PeakFTC2025 folder. Test and verify the code update by buliding the downloading into the robot.
    1. once changes is confirm the use following command to add the updates into the git repo and later commit the same updated files into the git repo
       git add "file_name"     where file_name = Name of the file that has been modified.
       git commit -m "commit_Message"  where commit_message = The information or message you want to give for future reference regarding this update
   2. Now push the commited code to git server by using the below command.
      git push


