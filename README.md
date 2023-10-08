# Projet d'ANIMATION 3D : Inverse Kinematics

Damien.Didier

Jules.Cassegrain

## Compilation

Pour compiler le projet il faut utiliser la commande suivante depuis la racine du projet:
```bash
mkdir build && cd build && cmake .. && make -j8
```
## Utilisation

Une fois l'executable lancé, bouger la sphére avec la GUI, puis appuyer sur le bouton "calculate IK" pour calculer la position des joints du squelette et voir le résultat.
## Travail réalisé

Nous avons utilisé comme base le code du TP05 car il contenait déja le code
pour le skinning et le skeleton.

Nous avons commencé par implémenter une sphére qui nous servirais de "target"
pour notre système d'IK. Cette sphére peut étre déplacé via la GUI.

Nous avons ensuite implémenté le système d'IK. nous avons donc créer une class IK_skeleton et une classe IK_skeleton_drawable afin de remplacer les classes skeleton et skeleton_drawable.

Dans IK_skeleton nous avons implémenté la fonction
```c++
void IK_skeleton::calculate_IK_joins(cgp::vec3 target_position)
```
qui implémente l'algorithme d'Inverse Kinematics [FABRIK](http://www.andreasaristidou.com/FABRIK.html) (Forward And Backward Reaching Inverse Kinematics) qui est un algorithme itératif permettant de calculer la position des joints d'un squelette en localisant des points sur une ligne.

Nous avons ensuite implémenté la fonction
```c++
void IK_skeleton::update_skeleton(float animation_time, skeleton_animation_structure &skeleton)
```
qui permet de mettre a jour la position des joints du squelette dans la géométrie locale du squelette aprés avoir calculé la position des joints via l'algorithme FABRIK. Cette fonction permet également calculer les matrice de transformation des joints du squelette.

Nous avons ensuite implémenté la classe IK_skeleton_drawable qui fonctionne de la même manière que la classe skeleton_drawable mais qui utilise un IK_skeleton a la place d'un skeleton.

Nous avons également changé le timer pour un timer_basic car nous avions besoin d'un timer qui ne bouclait pas une fois l'animation terminé et le timer_interval ne nous permettait pas de faire cela.

## Améliorations possibles

- Verifier si les joints et la targets ne sont pas alignés car cela peut poser des problémes pour converger vers la target (division par zéros) et dans le cas echéant ajouter un offset un des joints pour éviter ce probléme.
- Ajouter un systéme de contraintes pour les joints (par exemple un joint ne peut pas tourner a plus de 90° par rapport a son parent)
