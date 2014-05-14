rviz_pyplot
===========

Functions that allow simple plotting to rviz from Python.

Documentation is sparse, but to get started, try the following:

1. Start roscore
2. Start rviz
3. In rviz, under "Global Options", set the "Fixed Frame" to rviz_pyplot
4. Run the example scripts in the ```rviz_pyplot/examples``` folder. **Important**: You must run these scripts from inside a python interpreter (use ipython and the magic command "run"). If you try to run the scripts from the command line, the script exits before the message is sent and nothing will appear in rviz.
5. Now read the scripts to get an idea of what magic is happening there.
