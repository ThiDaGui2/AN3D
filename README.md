# Projet d'ANIMATION 3D : Inverse Kinematics

Damien.Didier

Jules.Cassegrain

## Présentation du projet

Pour le projet, nous avons décidé d'implémenter un système de cinématique inverse (IK)
sur une chaine linéaire d'articulations.

## Travail réalisé

Nous avons utilisé comme base le code du TP05 car il contenait déja le code
pour le skinning et le skeleton.

Nous avons commencé par ajouter à la scène une sphère qui nous servirait de "target"
pour notre système d'IK. Cette sphére peut étre déplacé via la GUI.

Nous avons ensuite implémenté le système d'IK. Pour cela, nous avons créer une classe 
`IK_skeleton` et une classe `IK_skeleton_drawable` afin de gérer la partie IK et mettre 
à jour les poses du skeleton_animation_structure.

Dans IK_skeleton nous avons implémenté la fonction

```c++
void IK_skeleton::calculate_IK_joints(cgp::vec3 target_position)
```

qui implémente l'algorithme d'Inverse Kinematics FABRIK [FABRIK](http://www.andreasaristidou.com/FABRIK.html)
(Forward And Backward Reaching Inverse Kinematics) qui est un algorithme itératif permettant de calculer la position 
des joints d'un squelette.

IK_skeleton a également la méthode

```c++
void IK_skeleton::update_skeleton(float animation_time, skeleton_animation_structure &skeleton)
```

qui calcule les rotations locales des jointures du squelette à appliquer pour coincider avec les positions 
globales calculées par `calculate_IK_Joints` et qui met a jour les poses du `skeleton_animation_structure`: 
en effet dans notre projet le `skeleton_animation_structure` utilisé ne possède que 1 ou 2 poses, la pose 
de départ (ou de repos) et la pose d'arrivée. À la fin de la méthode, la pose d'arrivée devient la nouvelle
pose de départ, et la pose calculée devient la nouvelle pose d'arrivée.

Nous avons ensuite implémenté la classe IK_skeleton_drawable qui fonctionne de la même manière que la 
classe skeleton_drawable mais qui utilise un IK_skeleton a la place d'un skeleton.

Enfin, nous avons changer le du timer de la scène, en passant sur un timer_basic, car nous le voulions 
pas que les animations bouclent.

## Difficultés rencontrées et résolutions

L'implémentation de l'algorithme FABRIK nous ne a pas posé de problème en particulier, a part la gestion du cas ou 
tous les joints et la ciblent se retrouvent alignés.
Pour corriger ce cas, après avoir vérifié l'alignement, nous décalons l'avant dernier joint du squelette d'un petit pas
dans une direction orthogonale à l'alignement.

La mise a jour des rotations du squelette nous a pris du temps, et l'implémentation n'est pas parfaite. En effet, dans 
certain cas il arrive que le squelette fasse une rotation complète sur lui même pour un petit changement sur la position
de la cible. L'algorithme se deroule ainsi: En commençant par le joint racine, 1) nous exprimons le joint fils dans la pose
IK dans le repère local du père, et 2) nous calculons la rotation à effectuer pour faire en sorte que le joints fils en
rest pose se retrouve colinéaire avec la position du joint dans la pose IK.

## Compilation

Pour compiler le projet il faut utiliser la commande suivante depuis la racine du projet:

```bash
mkdir build && cd build && cmake .. && make -j$(nproc)
```

## Utilisation

Une fois l'executable lancé, bouger la sphére avec la GUI, puis appuyer sur le bouton "calculate IK" pour calculer la position des joints du squelette et voir le résultat.

## Améliorations possibles

- Ajouter un systéme de contraintes pour les joints (par exemple un joint ne peut pas tourner a plus de 90° par rapport a son parent)
