# ddboat-sim
V-REP simulation of a DDBOAT.

Pour lancer la simulation: lancer roscore avant de lancer vrep. 
Pour contrôler les moteurs: soit utiliser RQT pour envoyer les commandes manuellement sur les topics /main/z_u1 et /main/z_u2, soit lancer boat.launch pour exécuter notre programme de contrôle. Dans src/ se trouvent les codes de contrôle du robot.
Pour visualiser les données des capteurs: afficher les topics associés dans RQT (/gyro, /acc, /gps)
/encoder ne fonctionne pas.

Modèle dynamique: Le corps du ddboat est modélisé par 4 boules (nous avons fait en sorte que le bateau pèse environ 2.5 kg), et les moteurs par deux autres boules reliées au corps par des joints.

Flottaison: la flottaison est codée par un Non-threaded child script qui est placé sur le corps principal du bateau et sur les deux moteurs. La formule a été trouvée sur github, et le niveau de l'eau est ajustable avec la variable waterLevel. 
Dans le dossier vrep/simuboat se trouvent nos tentatives pour simuler nous même la flottaison du ddboat. L'idée était de récupérer les données de la position, de la vitesse, de l'orientation et de la vitesse de rotation pour renvoyer ensuite une force et un couple modélisant la flottaison. Nous avons utilisé ROS pour cela. Pour lancer ce programme, il suffit d'exécuter mecaFlux.py. Le programme ne fonctionne pas (nous rencontrons des problèmes au niveau des fréquences d'émission et de réception de données).

Ajout de force: L'ajout de force se fait au niveau des deux boules qui modélisent les deux moteurs. Le bateau semble déséquilibré car il n'arrive pas à avancer en ligne droite lorsque nous appliquons la même force sur les deux moteurs. Cet ajout de forces se fait dans le fichier server_ros.lua.

Coque: Pour pouvoir afficher le bateau en 3D, nous avons utilisé la photogrammétrie. Cela consiste à prendre un grand nombre de photo (environ 70 dans notre cas) en variant légèrement l'angle de prise de vue à chaque photo pour permettre à un programme de reconstituer un objet 3D. Ensuite avec le logiciel blender on a pu remettre le modèle 3D à l'échelle et nettoyer certains bouts de la coque.

Contrôle par ROS: nous avons réalisé la structure ROS pour le contrôle du robot: le noeud boat_simuator est chargé de récupérer les données du ddboat, puis de les envoyer au noeud controller. Ce noeud reçoit la commande target (l'utilisateur choisit un point où doit aller le robot, par exemple via RQT avec le topic target). Nous n'avons pas encore eu le temps d'implémenter le contrôleur en lui même, les données de contrôle sont arbitraires pour le moment.

Capteurs: les envois des données des capteurs passent par des topics codés en lua (fichier server_ros.lua).
