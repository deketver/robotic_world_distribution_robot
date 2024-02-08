# Rust project
## Collection tool & Distribution robot

This repository show parts of the code done for a class of Advanced Programming, 2024.

The whole topic of the course was robotic world. Specifications defining how the robot's world should work and what it consists of were defined and unified across all teams.
My part in the project consisted of the Collection tool, which was made available for other teams to 'purchase'. This tool aimed to collect the desired Content
in the world if it met the user's specified criteria. The search was done via A\* algorithm and was therefore navigating the robot in the most energy-saving way.

Another part was an implementation of the Distribution robot (AI defining what the robot does in the world), which aimed to collect all Trees, Fish, and Rocks and distribute them equally into all available Markets in the world.
The distribution and which item belongs to which Market was done via Evolutionary algorithm with the goal to minimaze differences in the worth of different Markets. Each of the Contents
has its own value (similar to mass), which was defined in the world definition and also in the definition of the robot's score. This robot's behavior in the world maximizes the score.
Robot continues in the distribution of the Content until there is still any content to distribute in the world.

## Not compile
This codes serves only as a demonstration of the code part, but it was nested in the overall logic of the project code, for which is necessary an university account to compile. Therefore, this part of the code is not ment to be compiled.