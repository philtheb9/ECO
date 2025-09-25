Voici les bases d'un chargeur pour mesh GLTF en c++. Cette classe charge les meshs pas trop exotiques, avec animation par rig, bone, morp et axis (épée d'un guerrier).
Vous trouverez la classe OTextureList sur mon site : https://philthebjob.wixsite.com/moteur3d-eco.
Si votre systeme ne supporte pas les glBufferStorage de OpenGL 4.4, allez chercher le fichier ECO/OMesh_gltf old.
Prennez pour exemple Photo (photo.zip) pour un exemple d'utilisation complet, chargement et méthode d'affichage.

Here is the basic implementation of a GLTF mesh loader in C++. This class loads relatively standard mesh formats, including those with animations using rigs, bones, morph targets, and axis rotations (such as a warrior's sword). You can find the OTextureList class on my website: https://philthebjob.wixsite.com/3dengine-eco. If your system doesn't support glBufferStorage from OpenGL 4.4, use the ECO/OMesh_gltf_old file instead. For a complete example of usage, including loading and rendering, see the "Photo" example (photo.zip).


<img width="640" height="301" alt="photo_0" src="https://github.com/user-attachments/assets/cab1be46-d1c2-4714-bfda-42912308e5f4" />
