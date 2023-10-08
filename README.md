# Projet d'ANIMATION 3D : Inverse Kinematics

Damien.Didier

Jules.Cassegrain

## Introduction

Nous avons utilisé comme base le code du TP05 car il contenait déja le code
pour le skinning et le skeleton.

nous avons commencé par implémenter une sphére qui nous servirais de "target"
pour notre système d'IK. Cette sphére peut étre déplacé via la GUI.

nous avons ensuite implémenté le système d'IK. nous avons donc créer une class IK_skeleton et une classe IK_skeleton_drawable afin de remplacer les classes skeleton et skeleton_drawable.

Dans IK_skeleton nous avons implémenté la fonction
```c++
void IK_skeleton::calculate_IK_joins(cgp::vec3 target_position)
```
qui implémente l'algorithme d'Inverse Kinematics [FABRIK](http://www.andreasaristidou.com/FABRIK.html) (Forward And Backward Reaching Inverse Kinematics) qui est un algorithme itératif permettant de calculer la position des joints d'un squelette en localisant des points sur une ligne.

nous avons ensuite implémenté la fonction
```c++
void IK_skeleton::update_skeleton(float animation_time, skeleton_animation_structure &skeleton)
```
qui permet de mettre a jour la position des joints du squelette dans la géométrie globale et locale du squelette aprés avoir calculé la position des joints via l'algorithme FABRIK. Cette fonction permet également calculer les matrice de transformation des joints du squelette.


