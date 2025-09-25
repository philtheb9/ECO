
#include "OMesh_gltf.h"

// liste des textures des objets gltf
static OTextureList texturelist;

// les propriétés
static std::string  p_path = "donnees/mesh/";
static bool         p_mipmap = false;
static OVec3_f      p_scale(1.0f, 1.0f, 1.0f);
static int          p_filter = OTEXTURELIST_MESH;

static bool         p_info_anim = false;
static bool         p_info_bone = false;
static bool         p_info_hierarchy = false;
static bool         p_info_morph = false;

// --------------------
// --- constructeur ---
// --------------------

OMesh_gltf::OMesh_gltf()
{
    //clear();
}


// ----------------------
// --- le destructeur ---
// ----------------------

OMesh_gltf::~OMesh_gltf()
{
    clear();
}

// --------------------
// --- les méthodes ---
// --------------------

bool OMesh_gltf::anim_getAxis(std::vector<OMat4>& axis)
{
    if (m_animation_en_cours < 0) return false;

    uint64_t timeNow = ECO_getTicks();
    //uint64_t timeNow = ECO_getTimeInMilliseconde();
    float time = (float)(timeNow - m_start_time) / 1000.0f;

    if (time >= lst_animations[m_animation_en_cours].duration)
    {
        m_anim_loop--; // ici, pour faire au moins une animation, sans repetition
        // attention avec les animations en boucle > pensez à appeler anim_stop() à la fin du trajet/mouvement
        if (m_anim_loop > 0)
        {
            m_start_time = ECO_getTicks();
            //m_start_time = ECO_getTimeInMilliseconde();
            time = (float)(timeNow - m_start_time) / 1000.0f;
        }
        else
        {
            anim_stop();
            return false;
        }
    }

    // ---
    // --- recuperation des axes selon animations
    // ---
/*
animation -> channels
    > sampler       reference de la keyframe
    > target_node   bone/joint activé
    > target_path   mode (translation rotation scale weight)

animation -> samplers
    > std::vector<float> time       temps de déclenchemnt de la 'frame'
    > std::vector<float> value      valeurs à interpoler (en fonction de channels[].mode)
    > std::string interpolation     mode d'interpolation
*/
    //std::set<int> bone_anim;
    std::unordered_set<int> bone_anim; // plus rapide que set
    int idx_t0, idx_t1;
    float t_interpolation = 0.0f;
    OMat4 T,R,S;

    for (size_t i_cha = 0; i_cha < lst_animations[m_animation_en_cours].channels.size(); i_cha++)
    {
        // beaucoup de jonglage entre bone et joint
        int idx_bone = lst_animations[m_animation_en_cours].channels[i_cha].target_node;
        int idx_joint = lst_bones[idx_bone].index_joint; // -1 si non skinné
        int idx_sampler = lst_animations[m_animation_en_cours].channels[i_cha].sampler;
        s_gltf_animationSampler& sampler = lst_animations[m_animation_en_cours].samplers[idx_sampler];

        if (idx_joint < 0)
        {
            bone_anim.insert(idx_bone);

            //lst_bones[idx_bone].pos.set(0,0,0);
            //lst_bones[idx_bone].rot.set(0,0,0,1);
            lst_bones[idx_bone].scale.set(1,1,1);
        };

        // les deux keyframes entre lesquelles le temps actuel se situe
        // gérer cas: time <= premier keyframe ou >= dernier keyframe
        if (time <= sampler.time.front())
        {
            idx_t0 = 0; idx_t1 = 0;
        }
        else if (time >= sampler.time.back())
        {
            idx_t0 = (int)sampler.time.size() - 1;
            idx_t1 = idx_t0;
        }
        else
        {
            idx_t0 = -1; idx_t1 = -1;
            for (size_t i_tim = 1; i_tim < sampler.time.size(); ++i_tim)
            {
                if (sampler.time[i_tim] > time)
                {
                    idx_t1 = (int)i_tim;
                    idx_t0 = (int)i_tim - 1;
                    break;
                }
            }
            if (idx_t0 == -1 || idx_t1 == -1) continue;
        }
        // calcul interpolation
        if (idx_t0 != idx_t1)
            t_interpolation = (time - sampler.time[idx_t0]) / (sampler.time[idx_t1] - sampler.time[idx_t0]);
        else
            t_interpolation = 0.0f;

        // ---

        std::string code = lst_animations[m_animation_en_cours].channels[i_cha].target_path;
        if (code == "translation")
        {
            int v0 = idx_t0 * 3, v1 = idx_t1 * 3;
            OVec3_f t0,t1,val;

            switch(sampler.interpolation)
            {
            case GLTF_STEP:
                val.set(sampler.value[v0], sampler.value[v0+1], sampler.value[v0+2]);
                break;
            case GLTF_LINEAR:
                //t_interpolation = (time - sampler.time[idx_t0]) / (sampler.time[idx_t1] - sampler.time[idx_t0]);
                t0.set(sampler.value[v0], sampler.value[v0+1], sampler.value[v0+2]);
                t1.set(sampler.value[v1], sampler.value[v1+1], sampler.value[v1+2]);
                //
                val.lerp(t0,t1, t_interpolation);
                break;
            case GLTF_CUBICSPLINE:
                ECO_error_set("err CUBICSPLINE no implemented >OMesh_gltf.getAnimation");
                break;
            case GLTF_CATMULLROMSPLINE:
                ECO_error_set("err CATMULLROMSPLINE no implemented >OMesh_gltf.getAnimation");
                break;
            }
            if (idx_joint > -1) lst_joints[idx_joint].pos = val;
            else lst_bones[idx_bone].pos = val;

        }
        else if (code == "rotation")
        {
            int v0 = idx_t0 * 4;
            int v1 = idx_t1 * 4;
            OVec4_f t0,t1;
            OVec4_f val;

            switch(sampler.interpolation)
            {
            case GLTF_STEP:
                val.set(sampler.value[v0], sampler.value[v0+1], sampler.value[v0+2], sampler.value[v0+3]);
                break;
            case GLTF_LINEAR:
                //t_interpolation = (time - sampler.time[idx_t0]) / (sampler.time[idx_t1] - sampler.time[idx_t0]);
                t0.set(sampler.value[v0], sampler.value[v0+1], sampler.value[v0+2], sampler.value[v0+3]);
                t1.set(sampler.value[v1], sampler.value[v1+1], sampler.value[v1+2], sampler.value[v1+3]);
                //
                val.slerp(t0,t1, t_interpolation);
                break;
            case GLTF_CUBICSPLINE:
                ECO_error_set("err CUBICSPLINE no implemented >OMesh_gltf.getAnimation");
                break;
            case GLTF_CATMULLROMSPLINE:
                ECO_error_set("err CATMULLROMSPLINE no implemented >OMesh_gltf.getAnimation");
                break;
            }
            if (idx_joint > -1) lst_joints[idx_joint].rot = val;
            else lst_bones[idx_bone].rot = val;
        }
        else if (code == "scale")
        {
            int v0 = idx_t0 * 3;
            int v1 = idx_t1 * 3;
            OVec3_f t0,t1,val;

            switch(sampler.interpolation)
            {
            case GLTF_STEP:
                val.set(sampler.value[v0], sampler.value[v0+1], sampler.value[v0+2]);
                break;
            case GLTF_LINEAR:
                //t_interpolation = (time - sampler.time[idx_t0]) / (sampler.time[idx_t1] - sampler.time[idx_t0]);
                t0.set(sampler.value[v0], sampler.value[v0+1], sampler.value[v0+2]);
                t1.set(sampler.value[v1], sampler.value[v1+1], sampler.value[v1+2]);
                //
                val.lerp(t0,t1, t_interpolation);
                break;
            case GLTF_CUBICSPLINE:
                ECO_error_set("err CUBICSPLINE no implemented >OMesh_gltf.getAnimation");
                break;
            case GLTF_CATMULLROMSPLINE:
                ECO_error_set("err CATMULLROMSPLINE no implmented >OMesh_gltf.getAnimation");
                break;
            }
            if (idx_joint > -1) lst_joints[idx_joint].scale = val;
            else lst_bones[idx_bone].scale = val;
        }
        else if (code == "weights")
        {
            int morphCount = (int)(sampler.value.size() / sampler.time.size());
            if (morphCount <= 0) continue;

            // pour chaque meshPart cible
            for(auto& part : meshPart)
            //for (size_t i_par = 0; i_par < meshPart.size(); i_par++)
            // part = meshPart[i_par]
            {
                if (part.bone_ref != idx_bone) continue;
                // s'assurer que meshPart[mp].morph_weights a la bonne taille
                if ((int)part.morph_weights.size() < morphCount) part.morph_weights.resize(morphCount, 0.0f);
                // récupération/assignation
                if (sampler.interpolation == GLTF_STEP)
                {
                    // step prend la valeur de idx_t0
                    for (int i = 0; i < morphCount; ++i)
                    {
                        float w = sampler.value[idx_t0 * morphCount + i];
                        part.morph_weights[i] = w;
                    }
                }
                else // LINEAR (et autres éventuellement)
                {
                    for (int i = 0; i < morphCount; ++i)
                    {
                        float v0 = sampler.value[idx_t0 * morphCount + i];
                        float v1 = sampler.value[idx_t1 * morphCount + i];
                        float w = v0 + t_interpolation * (v1 - v0);
                        part.morph_weights[i] = w;
                    }
                }

                // normalisation > couteuse > en ayant des meshs corrects, on pourrait éviter cette phase
                float maxW = 0.0f;
                for (int i = 0; i < morphCount; i++) maxW = std::max(maxW, std::abs(part.morph_weights[i]));
                if (maxW > 1.0f + 1e-6f)
                {
                    // heuristique : si maxW <= 100 => exporter en pourcents (diviser par 100)
                    if (maxW <= 100.0f)
                    {
                        for (int i = 0; i < morphCount; i++) part.morph_weights[i] *= 0.01f;
                    }
                    else
                    {
                        // sinon on clamp pour éviter déformations extrêmes
                        for (int i = 0; i < morphCount; i++) part.morph_weights[i] = part.morph_weights[i] / maxW;
                    }
                }
            }

        }
    // end i_cha
    }

    // après = on ne calcule qu'une fois
    // axis_trs > initialisés au lancement d'une animation pour les bones qui servent directement à l'animation
    for (int i_bon : bone_anim)
    {
        T.loadIdentity();
        T.translate(lst_bones[i_bon].pos);
        R.loadIdentity();
        R.toMatrix(lst_bones[i_bon].rot);
        S.loadIdentity();
        S.scale(lst_bones[i_bon].scale); // initialisez à 1 ! *** y a t il des valeurs scale pour les animations de bone ?
        //
        lst_bones[i_bon].axis_trs = T*R*S;
    }
    for (size_t i_joi = 0; i_joi < lst_joints.size(); i_joi++)
    {
        T.loadIdentity();
        T.translate(lst_joints[i_joi].pos);
        R.loadIdentity();
        R.toMatrix(lst_joints[i_joi].rot);
        S.loadIdentity();
        S.scale(lst_joints[i_joi].scale);
        //
        lst_joints[i_joi].axis_computed = T*R*S;
    }

    // ---
    // --- création des matrices d'animations
    // ---

    // les bones définissent le nombre d'axes, leur nom, leur position, les enfants
    // les joints définissent le squelette, les axes qui servent à l'animation (il y en a moins)
    // load_skin lit les joints et leurs dépendances > transfère la liste des enfants_bone vers enfants_joints
    // >>> l'axisInverse (load_skin) a été transposée pour correspondre à ma matrice 'horizontale'

    axis.resize(lst_joints.size());
    axis[0].loadIdentity();
    _getAnim_axis(0, 0, axis);

    //#include <functional>
    // fonction dans une fonction = pour l'exemple
    std::function<void(int,const OMat4&)> propagateBone = [&](int idx, const OMat4& parent)
    {
        lst_bones[idx].axis_computed = parent * lst_bones[idx].axis_trs;
        for (int c : lst_bones[idx].children)
        {
            propagateBone(c, lst_bones[idx].axis_computed);
        }
    };

    if (bone_anim.size() > 0)
    {
        OMat4 identity(true);
        propagateBone(m_root_bone, identity);
    }

    return true;
}

void OMesh_gltf::_getAnim_axis(int idx_parent, int idx_children, std::vector<OMat4>& axis)
{
    axis[idx_children] = axis[idx_parent] * lst_joints[idx_children].axis_computed;

    // recupere la position (pose) pour placer les objets associés au perso
    lst_joints[idx_children].axis_computed = axis[idx_children];

    for (size_t i_chi = 0; i_chi < lst_joints[idx_children].children_joint.size(); i_chi++)
    {
        _getAnim_axis(idx_children, lst_joints[idx_children].children_joint[i_chi], axis);
    }

    // détail subtil et peu annoncé => APRèS la transmission de la position !!!
    axis[idx_children] *= lst_joints[idx_children].axis_inverse;
}

/*
anim morph est destiné à simplifier la gestion des morph : il n'y a pas besoin de shader spécifique
le calcul se faisant en cpu, il faut utiliser des morph légers et peu nombreux.
necessite openGL 4.4 (glBufferStorage) sinon voir OMesh_gltf old dans le dossier ECO/
*/
bool OMesh_gltf::anim_morph()
{
    bool anyUpload = false;

    for (auto& part : meshPart)
    {
        const size_t targets = part.morph_positions.size();
        if (targets == 0) continue;
        if (part.morph_weights.size() != targets) continue;

        const size_t vertexCount = part.vertex.size();
        if (vertexCount == 0) continue;

        // Vérifie rapidement si au moins un poids est significatif
        bool hasWork = false;
        for (float w : part.morph_weights)
        {
            if (std::abs(w) > 1e-6f) { hasWork = true; break; }
        }
        if (!hasWork) continue;

        // Accès direct aux buffers persistants
        OVec3_f* outPos = reinterpret_cast<OVec3_f*>(part.persistentPosBuffer);
        OVec3_f* outNor = reinterpret_cast<OVec3_f*>(part.persistentNormBuffer);

        const bool hasMorphNormals =
            !part.morph_normals.empty() &&
            part.morph_normals.size() == targets &&
            part.normal.size() == vertexCount;

        for (size_t i_ver = 0; i_ver < vertexCount; i_ver++)
        {
            OVec3_f pos = part.vertex[i_ver];
            OVec3_f nor = part.normal[i_ver];

            for (size_t i_tar = 0; i_tar < targets; i_tar++)
            {
                float w = part.morph_weights[i_tar];
                if (std::abs(w) < 1e-6f) continue;

                pos += part.morph_positions[i_tar][i_ver] * w;
                if (hasMorphNormals)
                {
                    nor += part.morph_normals[i_tar][i_ver] * w;
                }
            }

            if (hasMorphNormals)
            {
                // Normalisation seulement si vraiment utile
                float len2 = nor.length();
                if (std::abs(len2 - 1.0f) > 1e-3f)
                {
                    nor.normalize();
                }
            }

            if (outPos) outPos[i_ver] = pos;
            if (outNor) outNor[i_ver] = nor;
        }
        anyUpload = true;
    }

    return anyUpload;
}

bool OMesh_gltf::anim_running()
{
    return (m_animation_en_cours > -1);
}

bool OMesh_gltf::anim_start(std::string name, size_t loop)
{
    m_animation_en_cours = -1;

    // pour animation par bone
    for (auto& bone : lst_bones) bone.axis_trs = bone.axis_base;

    for (size_t i_ani = 0; i_ani < lst_animations.size(); i_ani++)
    {
        if (lst_animations[i_ani].name == name)
        {
            m_animation_en_cours = i_ani;
            m_start_time = ECO_getTicks();
            //m_start_time = ECO_getTimeInMilliseconde(); // > std::chrono
            m_anim_loop = loop;
            //ECO_error_set("start anim %s %d", name.c_str(), loop);
            break;
        }
    }
    return (m_animation_en_cours > -1);
}

bool OMesh_gltf::anim_start(size_t index, size_t loop)
{
    if (index > lst_animations.size()) return false;

    // pour animation par bone
    for (auto& bone : lst_bones) bone.axis_trs = bone.axis_base;

    m_animation_en_cours = index;
    m_start_time = ECO_getTicks();
    //m_start_time = ECO_getTimeInMilliseconde();
    m_anim_loop = loop;

    return true;
}

void OMesh_gltf::anim_stop()
{
    if (m_anim_neutral)
    {
        // retour à la position neutre > calcul lst_joints
        anim_start(0, 1);
        anim_getAxis(lst_neutral_axis);
    }

    m_animation_en_cours = -1;
    m_anim_loop = 0;
}




void OMesh_gltf::clear()
{
    m_path = "";
    m_file = "";
    m_num_instances = 0;
    //m_bound_min, m_bound_max;
    //m_scale = 1.0f;

    for (auto& part : meshPart)
    {
        if (part.persistentPosBuffer) glUnmapBuffer(GL_ARRAY_BUFFER);
        if (part.persistentNormBuffer) glUnmapBuffer(GL_ARRAY_BUFFER);

        if (part.vao_buffer[0] != 0) glDeleteBuffers(gltf_nbr_vao_buffer, part.vao_buffer);
        if (part.vao != 0) glDeleteVertexArrays(1, &part.vao);
        part.vao = 0;
    }

    material.clear();
    meshPart.clear();

    lst_bones.clear();
    lst_joints.clear();
    lst_animations.clear();
    lst_neutral_axis.clear();

    m_animation_en_cours = -1;
    m_start_time = 0;
    m_anim_loop = 0;
    m_anim_neutral = false;
    //m_root_joint = 0;
    //m_root_bone = 0;

    list_error.clear();
}

bool OMesh_gltf::create(std::string pathfile)
{
    std::string path;

    clear(); // indispensable > efface toutes les données (d'un précédent mesh)

    m_path = ECO_fileGetPath(p_path + pathfile);    //repertoire (pour trouver les textures et fichiers associés)
    m_file = ECO_fileGetName(pathfile);         //
    path = p_path + pathfile;   // le dossier de base (donnees/mesh/) + dossier éventuel du mesh (ex : objet/) & fichier

    tinygltf::Model model;

    if (!_load(model, path))
    {
        _setError(ERR_LOAD);
        return false;
    }
    if (!_load_material(model, path))
    {
        _setError(ERR_MATERIAL);
        return false;
    }
    if (!_load_bones(model))
    {
        _setError(ERR_BONES);
        return false;
    }
    if (!_load_skin(model)) // après bones
    {
        _setError(ERR_SKIN);
        return false;
    }
    if (!_load_geometry(model)) // après bones
    {
        _setError(ERR_GEOMETRY);
        return false;
    }
    if (!_load_animations(model)) // apres geometry / meshpart
    {
        _setError(ERR_ANIMATIONS);
        return false;
    }
    if (! _load_morphTargets(model))
    {
        _setError(ERR_MORPH);
        return false;
    }
    if (!_init(model))
    {
        _setError(ERR_INIT);
        return false;
    }

    // ---

    _make_vao();
    if (p_info_hierarchy) _getNodeHierarchy(model); // debug

    return true;
}




bool OMesh_gltf::_init(tinygltf::Model &model)
{
    // ---
    // --- bound du mesh
    // ---

    OVec3_f bmin, bmax;

    m_bound_min.set(10000,10000,10000);
    m_bound_max.set(-10000,-10000,-10000);

    for (const auto& mesh : model.meshes)
    {
        for (const auto& primitive : mesh.primitives)
        {
            const tinygltf::Accessor &accessor = model.accessors[primitive.attributes.find("POSITION")->second];

            bmin.set(accessor.minValues[0], accessor.minValues[1], accessor.minValues[2]);
            bmax.set(accessor.maxValues[0], accessor.maxValues[1], accessor.maxValues[2]);

            if (bmin.x < m_bound_min.x) m_bound_min.x = bmin.x;
            if (bmin.y < m_bound_min.y) m_bound_min.y = bmin.y;
            if (bmin.z < m_bound_min.z) m_bound_min.z = bmin.z;

            if (bmax.x > m_bound_max.x) m_bound_max.x = bmax.x;
            if (bmax.y > m_bound_max.y) m_bound_max.y = bmax.y;
            if (bmax.z > m_bound_max.z) m_bound_max.z = bmax.z;
        }
    }

    // ---
    // --- certaines échelles sont éronées
    // --- usuellement 1 pour 1 mètre dans gltf
    // ---

    m_scale = m_bound_min.length() + m_bound_max.length();
    if (m_scale < 0.1f)
    {
        _setError(ERR_SCALE, m_scale);
        m_scale = 1.0f / m_scale;
    }
    else
    {
        m_scale = 1.0f;
    }

    // ---
    // --- determination de la verticale (supposée) >> usuellement : y verticale
    // ---

    m_verticale = 1;
    if (m_bound_min.z > m_bound_min.y && m_bound_max.z > m_bound_max.y)
    {
        m_verticale = 2;
    }

    // ---
    // --- determination du bone_ref de chaque partie du mesh / matrice de base
    // ---
    for (size_t i_bon = 0; i_bon < lst_bones.size(); i_bon++)
    {
        for (size_t i_par = 0; i_par < meshPart.size(); i_par++)
        {
            if (lst_bones[i_bon].index_part == (int)i_par)
            {
                meshPart[i_par].bone_ref = i_bon;
                meshPart[i_par].axis_attachment = _getGlobalTransform(i_bon);
                break;
            }
        }
    }

    // ---
    // --- definition du mode d'animation
    // --- recuperation des bones servant d'axe d'animation
    // ---

    for (auto& part : meshPart)
    {
        part.animation_mode = GLTF_ANIM_NULL;
    }
    m_anim_neutral = false;

    if (lst_animations.size() == 0) return true;

    //
    for (auto& part : meshPart)
    {
        if (part.jointWeight.size() > 0)
        {
            part.animation_mode = GLTF_ANIM_RIG;
            //ECO_error_set("init %s > rig", meshPart[i_par].name.c_str());
        }
        else if (!part.morph_weights.empty())
        {
            part.animation_mode = GLTF_ANIM_MORPH;
            //ECO_error_set("init %s > morph", meshPart[i_par].name.c_str());
        }
    }

    // mesh animé sans joints, cas d'un coffre
    if (lst_joints.size() == 0)
    {
        // *** à améliorer
        _init_attach_bone();
    }
    //objets associés (épée casque ...) > ont été initialisés à GLTF_ANIM_NULL
    else
    {
        _init_attach_axis();
    }

    if (p_info_anim)
    {
        ECO_error_set(" ");
        for (auto& part : meshPart)
        {
            ECO_error_set("%s > %s / %d", part.name.c_str(), gltf_mode_anim[part.animation_mode].c_str(), part.bone_ref);
        }
    }

    // ---
    // --- position neutre
    // ---

    anim_start(0, 1);
    anim_getAxis(lst_neutral_axis);
    anim_stop();
    m_anim_neutral = true;

    return true;
}

void OMesh_gltf::_init_attach_axis()
{
    for (size_t i_bon = 0; i_bon < lst_bones.size(); i_bon++)
    {
        int i_par = lst_bones[i_bon].index_part;
        bool ok = false;
        int current = i_bon;

        if (i_par < 0 || i_par >= static_cast<int>(meshPart.size())) continue;
        if (meshPart[i_par].animation_mode != GLTF_ANIM_NULL) continue;

        meshPart[i_par].bone_ref = i_bon; // rattache ce mesh à son node par defaut
        meshPart[i_par].animation_mode = GLTF_ANIM_BONE;
        //meshPart[i_par].axis_attachment = lst_bones[i_bon].axis_base;

        // Trouver le joint parent
        while (current >= 0)
        {
            int parent = -1;
            for (size_t i_parent = 0; i_parent < lst_bones.size(); i_parent++)
            {
                for (size_t i_chi = 0; i_chi < lst_bones[i_parent].children.size(); i_chi++)
                {
                    if (lst_bones[i_parent].children[i_chi] == current)
                    {
                        parent = i_parent;
                        break;
                    }
                }

                if (parent > -1 && lst_bones[current].index_joint > -1)
                {
                    meshPart[i_par].axis_attachment = lst_bones[i_bon].axis_base;
                    meshPart[i_par].joint_ref = lst_bones[current].index_joint;
                    meshPart[i_par].animation_mode = GLTF_ANIM_AXIS;
                    //meshPart[i_par].bone_ref = -1; // il y a un joint, donc pas de bone
                    ok = true;
                    break;
                }
            }
            if (ok) break;
            current = parent;
        }
    }
}

// il y a des bones, une animation, mais pas de joint/rig
void OMesh_gltf::_init_attach_bone()
{
    lst_joints.resize(lst_bones.size());

    for (size_t i_bon = 0; i_bon < lst_bones.size(); i_bon++)
    {
        lst_bones[i_bon].index_joint = i_bon;

        lst_joints[i_bon].axis_inverse = lst_bones[i_bon].axis_base.getInverse();
        lst_joints[i_bon].axis_inverse.transpose(); // ma matrice 'horizontale'

        for (size_t i = 0; i < lst_bones[i_bon].children.size(); i++)
        {
            lst_joints[i_bon].children_bone.push_back( lst_bones[i_bon].children[i] );
        }
    }

    for (size_t i_par = 1; i_par < meshPart.size(); i_par++)
    {
        if (meshPart[i_par].animation_mode != GLTF_ANIM_NULL) continue;

        meshPart[i_par].animation_mode = GLTF_ANIM_AXIS;
        meshPart[i_par].axis_attachment.loadIdentity();
        meshPart[i_par].joint_ref = 1; // meshPart[i_par].bone_ref; // ***
    }
}

OMat4 OMesh_gltf::_getGlobalTransform(int nodeIndex)
{
    OMat4 globalTransform;
    bool ok = true;

    globalTransform.loadIdentity();
    globalTransform = lst_bones[nodeIndex].axis_base * globalTransform;

    while(ok)
    {
        ok = false;
        // cherche le parent
        for (size_t i_bon = 0; i_bon < lst_bones.size(); i_bon++)
        {
            for (size_t i_chi = 0; i_chi < lst_bones[i_bon].children.size(); i_chi++)
            {
                if (lst_bones[i_bon].children[i_chi] == nodeIndex)
                {
                    // recupere la position parent
                    nodeIndex = i_bon;
                    globalTransform = lst_bones[nodeIndex].axis_base * globalTransform;
                    ok = true;
                    break;
                }
            }
            if (ok) break;
        }
    }

    return globalTransform;
}








void OMesh_gltf::draw()
{
    // pour une texture unique !
    for (auto& part : meshPart)
    {
        glBindVertexArray(part.vao);
        glDrawElements(GL_TRIANGLES, part.indice.size(), GL_UNSIGNED_INT, 0);
    }
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    glBindTexture(GL_TEXTURE_2D, 0);
    glActiveTexture(GL_TEXTURE0);
}

void OMesh_gltf::draw(size_t part)
{
    glBindVertexArray(meshPart[part].vao);
    glDrawElements(GL_TRIANGLES, meshPart[part].indice.size(), GL_UNSIGNED_INT, 0);

    // + end_draw_part() !!!
}

void OMesh_gltf::draw_line(int type)
{
    //GL_LINES GL_LINE_LOOP GL_LINE_STRIP
    for (auto& part : meshPart)
    {
        glBindVertexArray(part.vao);
        glDrawElements(type, part.indice.size(), GL_UNSIGNED_INT, 0);
    }

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void OMesh_gltf::draw_line(size_t part, int type)
{
    //GL_LINES GL_LINE_LOOP GL_LINE_STRIP
    glBindVertexArray(meshPart[part].vao);
    glDrawElements(type, meshPart[part].indice.size(), GL_UNSIGNED_INT, 0);

    // + end_draw_part() !!!
}

void OMesh_gltf::draw_multi(size_t part)
{
    glBindVertexArray(meshPart[part].vao);
    glDrawElementsInstanced(GL_TRIANGLES, meshPart[part].indice.size(), GL_UNSIGNED_INT, 0, m_num_instances);

    //désactivation du VAO dans end_draw_part()
}

void OMesh_gltf::draw_multi_line(int type)
{
    for (auto& part : meshPart)
    {
        glBindVertexArray(part.vao);
        glDrawElementsInstanced(type, part.indice.size(), GL_UNSIGNED_INT, 0, m_num_instances);
    }
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void OMesh_gltf::draw_multi_line(size_t part, int type)
{
    glBindVertexArray(meshPart[part].vao);
    glDrawElementsInstanced(type, meshPart[part].indice.size(), GL_UNSIGNED_INT, 0, m_num_instances);

    //désactivation du VAO dans end_draw_part()
}

void OMesh_gltf::end_draw_part()
{
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    glBindTexture(GL_TEXTURE_2D, 0);
    glActiveTexture(GL_TEXTURE0);
}





// --- get animation
int OMesh_gltf::getAnimation_mode(size_t part) const
{
    return meshPart[part].animation_mode;
}

const std::vector<s_gltf_animation>& OMesh_gltf::getAnimations() const
{
    return lst_animations;
}

// recherche de l'index par le nom
int OMesh_gltf::getAnimation(std::string ref) const
{
    for (size_t i_ani = 0; i_ani < lst_animations.size(); i_ani++)
    {
        if (lst_animations[i_ani].name == ref)
        {
            return i_ani;
        }
    }

    return -1;
}

// recherche du nom par l'index (boucle)
std::string OMesh_gltf::getAnimation(size_t ref) const
{
    if (ref >= lst_animations.size()) return "";

    return lst_animations[ref].name;
}

float OMesh_gltf::getAnimation_duration() const
{
    //if (m_animation_en_cours < 0) return;
    return lst_animations[m_animation_en_cours].duration;
}

size_t OMesh_gltf::getAnimation_size() const
{
    return lst_animations.size();
}





// plutot opacity
float OMesh_gltf::getAlpha(size_t part) const
{
    return material[ meshPart[part].material ].opacity;
}

// recuperation de l'axe reference pour un equipement
const OMat4& OMesh_gltf::getAxisAttach(size_t idx_joint) const
{
    return lst_joints[idx_joint].axis_computed;
}

// recuperation pour la pose neutre et passage dans le shader
const std::vector<OMat4>& OMesh_gltf::getAxisNeutral() const
{
    return lst_neutral_axis;
}

const OVec4_f& OMesh_gltf::getBaseColor(size_t part) const
{
    return material[ meshPart[part].material ].baseColor;
}

bool OMesh_gltf::getBone(std::string name, s_gltf_bone& result) const
{
    for (auto& bone : lst_bones)
    {
        if (bone.name == name)
        {
            result = bone;
            return true;
        }
    }
    return false;
}

// pour les bones animés > sans reference (pointeur) pour etre identique à getJointOfPart
const OMat4 OMesh_gltf::getBoneOfPart(size_t part) const
{
    //meshPart[part].animation_mode == GLTF_ANIM_BONE)
    return lst_bones[meshPart[part].bone_ref].axis_computed;
}

const OVec3_f& OMesh_gltf::getBound_min(int part) const
{
    if (part == -1)
    {
        return m_bound_min;
    }

    return meshPart[part].bound_min;
}

const OVec3_f& OMesh_gltf::getBound_max(int part) const
{
    if (part == -1)
    {
        return m_bound_max;
    }

    return meshPart[part].bound_max;
}

const OVec4_f& OMesh_gltf::getDiffuse(size_t part) const
{
    return material[ meshPart[part].material ].diffuseColor;
}

const OVec4_f& OMesh_gltf::getEmissive(size_t part) const
{
    return material[ meshPart[part].material ].emissive;
}




uint32_t OMesh_gltf::getId_diffuseMap(size_t part) const
{
    return material[ meshPart[part].material ].tex_diffuse;
}

uint32_t OMesh_gltf::getId_emissiveMap(size_t part) const
{
    return material[ meshPart[part].material ].tex_emissive;
}

uint32_t OMesh_gltf::getId_normalMap(size_t part) const
{
    return material[ meshPart[part].material ].tex_normal;
}

uint32_t OMesh_gltf::getId_occlusionMap(size_t part) const
{
    return material[ meshPart[part].material ].tex_occlusion;
}

uint32_t OMesh_gltf::getId_specularMap(size_t part) const
{
    return material[ meshPart[part].material ].tex_specular;
}
//
uint32_t OMesh_gltf::getId_albedoMap(size_t part) const
{
    return material[ meshPart[part].material ].tex_diffuse;
}

uint32_t OMesh_gltf::getId_metallicMap(size_t part) const
{
    return material[ meshPart[part].material ].tex_specular;
}

uint32_t OMesh_gltf::getId_roughnessMap(size_t part) const
{
    return material[ meshPart[part].material ].tex_roughness;
}

uint32_t OMesh_gltf::getId_AOMap(size_t part) const
{
    return material[ meshPart[part].material ].tex_occlusion;
}





bool OMesh_gltf::getJoint(std::string name, s_gltf_joint& result) const
{
    for (auto& joint : lst_joints)
    {
        if (joint.name == name)
        {
            result = joint;
            return true;
        }
    }
    return false;
}

// joint_ref n'est défini que pour les éléments associés > épée d'un guerrier par exemple
OMat4 OMesh_gltf::getJointOfPart(size_t part) const
{
    int idx_joint = meshPart[part].joint_ref;
    //if (idx_joint >= 0 && idx_joint < (int)lst_joints.size())
    return lst_joints[idx_joint].axis_computed * meshPart[part].axis_attachment;

    //OMat4 result(true);
    //return result; //OMat4::Identity();
}

const s_gltf_material& OMesh_gltf::getMaterial(size_t idx) const
{
    return material[idx];
}

size_t OMesh_gltf::getMaterial_size() const
{
    return material.size();
}

// debug = hierarchie des nodes
void OMesh_gltf::_getNodeHierarchy(const tinygltf::Model &model)
{
    //std::vector<std::string> path;
    std::vector<int> bone;

    ECO_error_set("mesh : %s", m_file.c_str());

    for (size_t i_bon = 0; i_bon < model.nodes.size(); i_bon++)
    {
        const auto &node = model.nodes[i_bon];
        ECO_error_set("bone %d %s", i_bon, node.name.c_str());
    }
    ECO_error_set(">>>");

    for (size_t i_bon = 0; i_bon < model.nodes.size(); i_bon++)
    {
        //path.clear();
        bone.clear();
        int current = i_bon;
        while (current >= 0)
        {
            //const auto &node = model.nodes[current];
            //path.push_back(node.name.empty() ? ("<unnamed_" + std::to_string(current) + ">") : node.name);
            bone.push_back(current);

            // Trouver le parent
            int parent = -1;
            for (size_t i = 0; i < model.nodes.size(); i++)
            {
                for (auto child : model.nodes[i].children)
                {
                    if (child == current)
                    {
                        parent = static_cast<int>(i);
                        break;
                    }
                }
                if (parent >= 0) break;
            }
            current = parent;
        }

        // Inverser le chemin (on a collecté de bas en haut)
        //std::reverse(path.begin(), path.end());
        std::reverse(bone.begin(), bone.end());

        // Construire une string style "Root -> Armature -> Hand -> WeaponSocket -> Sword"
        std::string result = ECO_intToStr(i_bon) + " >> ";
        for (size_t i = 0; i < bone.size(); i++)
        {
            //result += path[i] + "(" + bone[i] + ")";
            //if (i < path.size() - 1) result += " -> ";
            result += ECO_intToStr(bone[i]) + " (" + ECO_intToStr(lst_bones[bone[i]].index_joint) + ")";
            if (i < bone.size() - 1) result += "-> ";
        }
        ECO_error_set("%s", result.c_str());
    }
    ECO_error_set("---");
}

// opacity plutot que alpha
float OMesh_gltf::getOpacity(size_t part) const
{
    return material[ meshPart[part].material ].opacity;
}

const s_gltf_part& OMesh_gltf::getPart(size_t part) const
{
    return meshPart[part];
}

std::string OMesh_gltf::getPart_name(size_t part) const
{
    return meshPart[part].name;
}

size_t OMesh_gltf::getPart_size() const
{
    return meshPart.size();
}

bool OMesh_gltf::getPart_visibility(size_t part) const
{
    return meshPart[part].visibility;
}

std::string OMesh_gltf::getPath() const
{
    return m_path;
}

float OMesh_gltf::getRoughness_factor(size_t part) const
{
    return material[ meshPart[part].material ].roughness_factor;
}

float OMesh_gltf::getScale() const
{
    return m_scale;
}

const OVec4_f& OMesh_gltf::getSpecular(size_t part) const
{
    return material[ meshPart[part].material ].specular;
}

float OMesh_gltf::getSpecular_factor(size_t part) const
{
    return material[ meshPart[part].material ].specular_factor;
}

size_t OMesh_gltf::getVerticale() const
{
    return m_verticale; // x=0 y=1 z=2
}







bool OMesh_gltf::_load(tinygltf::Model& model, std::string& pathfile)
{
    tinygltf::TinyGLTF filegftf;
    std::string err;
    std::string warn;
    bool ret = false;
    std::string ext = ECO_fileGetExt(pathfile);

    // ------------------------------
    // --- chargement des données ---
    // ------------------------------

    if (ext == "glb")
    {
        ret = filegftf.LoadBinaryFromFile(&model, &err, &warn, pathfile.c_str());
    }
    else
    {
        ret = filegftf.LoadASCIIFromFile(&model, &err, &warn, pathfile.c_str());
    }
    if (!warn.empty())
    {
        _setError(ERR_LOAD0, warn.c_str());
        return false;
    }
    if (!err.empty())
    {
        _setError(ERR_LOAD1, pathfile.c_str());
        return false;
    }
    if (!ret)
    {
        _setError(ERR_LOAD2, pathfile.c_str());
        return false;
    }

    return true;
}

bool OMesh_gltf::_load_animations(tinygltf::Model &model)
{
    std::string str;

    if (p_info_anim)
    {
        ECO_error_set("");
        ECO_error_set(m_file.c_str());
        ECO_error_set("animations size : %d", model.animations.size());
        ECO_error_set("---");
    }

    lst_animations.resize(model.animations.size()); // indispensable

    for (size_t i_ani = 0; i_ani < model.animations.size(); i_ani++)
    {
        const auto &animation = model.animations[i_ani];

        lst_animations[i_ani].name = animation.name;
        if (p_info_anim)
        {
            ECO_error_set("Animation: %s", lst_animations[i_ani].name.c_str());
        }

        for (const auto &channel : animation.channels)
        {
            s_gltf_animationChannel anim_channel;

            anim_channel.sampler = channel.sampler;
            anim_channel.target_node = channel.target_node;
            anim_channel.target_path = channel.target_path;
            lst_animations[i_ani].channels.push_back(anim_channel);

            if (p_info_anim)
            {
                ECO_error_set("");
                ECO_error_set("Channel targeting node: %d >j %d", channel.target_node, lst_bones[channel.target_node].index_joint );
                ECO_error_set(" > Target path: %s", channel.target_path.c_str());
            }
            s_gltf_animationSampler anim_sampler;

            const auto &sampler = animation.samplers[channel.sampler];
            const auto &inputAccessor = model.accessors[sampler.input];
            const auto &outputAccessor = model.accessors[sampler.output];

            const auto &inputBufferView = model.bufferViews[inputAccessor.bufferView];
            const auto &inputBuffer = model.buffers[inputBufferView.buffer];
            const float* inputData = reinterpret_cast<const float *>(&inputBuffer.data[inputBufferView.byteOffset + inputAccessor.byteOffset]);

            anim_sampler.time.resize(inputAccessor.count);
            anim_sampler.time.assign(inputData, inputData + inputAccessor.count);

            const auto &outputBufferView = model.bufferViews[outputAccessor.bufferView];
            const auto &outputBuffer = model.buffers[outputBufferView.buffer];

            if (channel.target_path == "translation")
            {
                const float *outputData = reinterpret_cast<const float*>(&outputBuffer.data[outputBufferView.byteOffset + outputAccessor.byteOffset]);

                anim_sampler.value.resize(inputAccessor.count*3);
                anim_sampler.value.assign(outputData, outputData + outputAccessor.count*3);
            }
            else if (channel.target_path == "rotation")
            {
                const float *outputData = reinterpret_cast<const float*>(&outputBuffer.data[outputBufferView.byteOffset + outputAccessor.byteOffset]);

                anim_sampler.value.resize(inputAccessor.count*4);
                anim_sampler.value.assign(outputData, outputData + outputAccessor.count*4);
            }
            else if (channel.target_path == "scale")
            {
                const float *outputData = reinterpret_cast<const float*>(&outputBuffer.data[outputBufferView.byteOffset + outputAccessor.byteOffset]);

                anim_sampler.value.resize(inputAccessor.count*3);
                anim_sampler.value.assign(outputData, outputData + outputAccessor.count*3);
            }
            else if (channel.target_path == "weights")
            {
                const float *outputData = reinterpret_cast<const float*>(&outputBuffer.data[outputBufferView.byteOffset + outputAccessor.byteOffset]);

                anim_sampler.value.resize(inputAccessor.count);
                anim_sampler.value.assign(outputData, outputData + outputAccessor.count);
            }

            if (sampler.interpolation == "STEP") anim_sampler.interpolation = GLTF_STEP;
            else if (sampler.interpolation == "LINEAR") anim_sampler.interpolation = GLTF_LINEAR;
            else if (sampler.interpolation == "CUBICSPLINE") anim_sampler.interpolation = GLTF_CUBICSPLINE;
            else anim_sampler.interpolation = GLTF_CATMULLROMSPLINE;
            //
            lst_animations[i_ani].samplers.push_back(anim_sampler);

            //durée de l'animation
            //if (lst_animations[i_ani].duration < anim_sampler.time[anim_sampler.time.size()-1])
            //{
                lst_animations[i_ani].duration = anim_sampler.time[anim_sampler.time.size()-1];
            //}
        // end for channel
        }
        if (p_info_anim)
        {
            ECO_error_set("Animation: %s / %f s", lst_animations[i_ani].name.c_str(), lst_animations[i_ani].duration);
        }
    }

    return true;
}

bool OMesh_gltf::_load_geometry(tinygltf::Model& model)
{
    size_t i_par = 0;

    for (size_t i_gltf = 0; i_gltf < model.meshes.size(); i_gltf++)
    {
        const auto &mesh = model.meshes[i_gltf];

        for (size_t i_pri = 0; i_pri < mesh.primitives.size(); i_pri++)
        {
            const tinygltf::Primitive &primitive = mesh.primitives[i_pri];

            // pour chaque mesh et chaque partie du mesh
            i_par = meshPart.size();
            meshPart.push_back(s_gltf_part() );

            if (mesh.name.empty())
            {
                meshPart[i_par].name = "mesh_" + ECO_intToStr(i_gltf);
            }
            else
            {
                meshPart[i_par].name = mesh.name;
            }
            //ECO_error_set("%s", mesh.name.c_str());
            //meshPart[i_par].topologie_style = TOP_TRIANGLE; // toujours
            meshPart[i_par].material = primitive.material;

            if (primitive.attributes.find("POSITION") != primitive.attributes.end())
            {
                const tinygltf::Accessor &accessor = model.accessors[primitive.attributes.find("POSITION")->second];
                const tinygltf::BufferView &bufferView = model.bufferViews[accessor.bufferView];
                const tinygltf::Buffer &buffer = model.buffers[bufferView.buffer];

                const float *positions = reinterpret_cast<const float*>(&(buffer.data[bufferView.byteOffset + accessor.byteOffset]) );

                meshPart[i_par].vertex.reserve(accessor.count);
                for (size_t i_ver = 0; i_ver < accessor.count; i_ver++)
                {
                    meshPart[i_par].vertex.push_back( OVec3_f(positions[i_ver*3], positions[i_ver*3 +1], positions[i_ver*3 +2]) );
                }
                //ECO_error_set("%s v %d", m_file.c_str(), meshPart[i_par].vertex.size());
            }

            if (primitive.attributes.find("NORMAL") != primitive.attributes.end())
            {
                const tinygltf::Accessor &accessor = model.accessors[primitive.attributes.find("NORMAL")->second];
                const tinygltf::BufferView &bufferView = model.bufferViews[accessor.bufferView];
                const tinygltf::Buffer &buffer = model.buffers[bufferView.buffer];

                const float *normals = reinterpret_cast<const float*>(&(buffer.data[bufferView.byteOffset + accessor.byteOffset]));

                meshPart[i_par].normal.reserve(accessor.count);
                for (size_t i_nor = 0; i_nor < accessor.count; i_nor++)
                {
                    meshPart[i_par].normal.push_back( OVec3_f(normals[i_nor*3], normals[i_nor*3 +1], normals[i_nor*3 +2]) );
                }
                //ECO_error_set("n %d", meshPart[i_par].normal.size());
            }

            if (primitive.attributes.find("TEXCOORD_0") != primitive.attributes.end())
            {
                const tinygltf::Accessor &accessor = model.accessors[primitive.attributes.find("TEXCOORD_0")->second];
                const tinygltf::BufferView &bufferView = model.bufferViews[accessor.bufferView];
                const tinygltf::Buffer &buffer = model.buffers[bufferView.buffer];

                const float *uvs = reinterpret_cast<const float*>(&(buffer.data[bufferView.byteOffset + accessor.byteOffset]));

                meshPart[i_par].uv.reserve(accessor.count);
                for (size_t i_uv = 0; i_uv < accessor.count; i_uv++)
                {
                    meshPart[i_par].uv.push_back( OVec2_f(uvs[i_uv*2], uvs[i_uv*2 +1]) );
                }
                //ECO_error_set("t %d", meshPart[i_par].uv.size());
            }

            // faces indices = sommet
            if (primitive.indices > 0)
            {
                const tinygltf::Accessor &indexAccessor = model.accessors[primitive.indices];
                const tinygltf::BufferView &indexBufferView = model.bufferViews[indexAccessor.bufferView];
                const tinygltf::Buffer &indexBuffer = model.buffers[indexBufferView.buffer];

                if (indexAccessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT)
                {
                    const unsigned short *indices = reinterpret_cast<const unsigned short*>(&(indexBuffer.data[indexBufferView.byteOffset + indexAccessor.byteOffset]));

                    meshPart[i_par].indice.reserve(indexAccessor.count);
                    for (size_t i_ind = 0; i_ind < indexAccessor.count; i_ind += 3)
                    {
                        meshPart[i_par].indice.push_back(indices[i_ind]);
                        meshPart[i_par].indice.push_back(indices[i_ind+1]);
                        meshPart[i_par].indice.push_back(indices[i_ind+2]);
                    }
                }
                else if (indexAccessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT)
                {
                    const unsigned int *indices = reinterpret_cast<const unsigned int*>(&(indexBuffer.data[indexBufferView.byteOffset + indexAccessor.byteOffset]));

                    meshPart[i_par].indice.reserve(indexAccessor.count);
                    for (size_t i_ind = 0; i_ind < indexAccessor.count; i_ind += 3)
                    {
                        meshPart[i_par].indice.push_back(indices[i_ind]);
                        meshPart[i_par].indice.push_back(indices[i_ind+1]);
                        meshPart[i_par].indice.push_back(indices[i_ind+2]);
                    }
                }
                else
                {
                    ECO_error_set("GLTF %s erreur indice", m_file.c_str());
                }
            // end primitive indice
            }

            if (primitive.attributes.find("JOINTS_0") != primitive.attributes.end() &&
                primitive.attributes.find("WEIGHTS_0") != primitive.attributes.end())
            {
                // Obtenir les indices des joints (JOINTS_0)
                const tinygltf::Accessor& jointsAccessor = model.accessors[primitive.attributes.at("JOINTS_0")];
                const tinygltf::BufferView& jointsBufferView = model.bufferViews[jointsAccessor.bufferView];
                const uint8_t* jointsData = &model.buffers[jointsBufferView.buffer].data[jointsBufferView.byteOffset + jointsAccessor.byteOffset];

                // Obtenir les poids (WEIGHTS_0)
                const tinygltf::Accessor& weightsAccessor = model.accessors[primitive.attributes.at("WEIGHTS_0")];
                const tinygltf::BufferView& weightsBufferView = model.bufferViews[weightsAccessor.bufferView];
                const uint8_t* weightsData = &model.buffers[weightsBufferView.buffer].data[weightsBufferView.byteOffset + weightsAccessor.byteOffset];

                // Nombre de poids par vertex
                int numWeightsPerVertex = weightsAccessor.type == TINYGLTF_TYPE_VEC4 ? 4 : 3; // normalement 4
                if (numWeightsPerVertex != 4) ECO_error_set("%d numWeightsPerVertex %d", m_file.c_str(), numWeightsPerVertex);

                size_t numVertices = jointsAccessor.count;
                meshPart[i_par].jointWeight.reserve(numVertices);
                meshPart[i_par].weight.reserve(numVertices);

                // Les indices des joints pour chaque vertex (4 bones & 4 weights par vertex)
                if (jointsAccessor.type == TINYGLTF_PARAMETER_TYPE_SHORT)
                {
                    for (size_t i_ver = 0; i_ver < numVertices; i_ver++)
                    {
                        uint16_t* vertexJoints = (uint16_t*)(jointsData + i_ver * numWeightsPerVertex * sizeof(uint16_t));
                        meshPart[i_par].jointWeight.push_back( OVec4_ui(vertexJoints[0], vertexJoints[1], vertexJoints[2], vertexJoints[3]) );
                    }
                }
                else
                {
                    for (size_t i_ver = 0; i_ver < numVertices; i_ver++)
                    {
                        uint8_t* vertexJoints = (uint8_t*)(jointsData + i_ver * numWeightsPerVertex * sizeof(uint8_t));
                        meshPart[i_par].jointWeight.push_back( OVec4_ui(vertexJoints[0], vertexJoints[1], vertexJoints[2], vertexJoints[3]) );
                    }
                }

                // Les poids pour chaque vertex
                for (size_t i_ver = 0; i_ver < numVertices; i_ver++)
                {
                    if (weightsAccessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT)
                    {
                        float* vertexWeights = (float*)(weightsData + i_ver * numWeightsPerVertex * sizeof(float));
                        meshPart[i_par].weight.push_back( OVec4_f(vertexWeights[0], vertexWeights[1], vertexWeights[2], vertexWeights[3]) );
                    }
                    else if (weightsAccessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE) // normalisé (0 - 255 -> 0.0 - 1.0)
                    {
                        uint8_t* vertexWeights = (uint8_t*)(weightsData + i_ver * numWeightsPerVertex * sizeof(uint8_t));
                        meshPart[i_par].weight.push_back( OVec4_f(
                            vertexWeights[0] / 255.0f,
                            vertexWeights[1] / 255.0f,
                            vertexWeights[2] / 255.0f,
                            vertexWeights[3] / 255.0f
                        ));
                    }
                    else if (weightsAccessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT) // normalisé (0 - 65535 -> 0.0 - 1.0)
                    {
                        uint16_t* vertexWeights = (uint16_t*)(weightsData + i_ver * numWeightsPerVertex * sizeof(uint16_t));
                        meshPart[i_par].weight.push_back( OVec4_f(
                            vertexWeights[0] / 65535.0f,
                            vertexWeights[1] / 65535.0f,
                            vertexWeights[2] / 65535.0f,
                            vertexWeights[3] / 65535.0f
                        ));
                    }
                }
            }
        // end i_pri
        }
        // normalisation
        /*
        for (auto& w : meshPart[i_par].weight)
        {
            float weightSum = w.x + w.y + w.z + w.w;
            if (fabs(weightSum) > 0.0f && fabs(weightSum - 1.0f) > 0.001f)
            {
                w.x /= weightSum;
                w.y /= weightSum;
                w.z /= weightSum;
                w.w /= weightSum;
            }
        }
        */

    // end i_gltf
    }

    return true;
}

bool OMesh_gltf::_load_bones(tinygltf::Model& model)
{
    OVec3_f pos;
    OVec4_f rot;
    OVec3_f scale;
    OMat4 T,R,S;

    m_root_joint = 0;
    m_root_bone = 0;
    if (model.skins.size() > 0)
    {
        m_root_joint = model.skins[0].joints[0];
    }

    if (p_info_bone)
    {
        ECO_error_set(" ");
        ECO_error_set("file  %s root %d", m_file.c_str(), m_root_joint);
        ECO_error_set(">bones>");
    }

    //ECO_error_set("file %s", m_file.c_str());
    lst_bones.reserve(model.nodes.size());
    for (size_t i_nod = 0; i_nod < model.nodes.size(); i_nod++)
    {
        const tinygltf::Node &node = model.nodes[i_nod];

        s_gltf_bone bone;
        bone.name = node.name;
        bone.children = node.children;
        bone.index_part = node.mesh;
        bone.axis_base.loadIdentity();

        // position de base > attachement
        // utilisé pour les parties additionnelles (hache du barbare) pour la décaler en fonction de l'os/joint
        if (!node.matrix.empty())
        {
            for (int i = 0; i < 16; i++)
            {
                //matrix[i / 4][i % 4] = static_cast<float>(node.matrix[i]);
                bone.axis_base[i] = static_cast<float>(node.matrix[i]);
            }
            //ECO_error_set("mat4 > %s > %s", bone.name.c_str(), node.mesh);
        }
        else
        {
            pos.set(0,0,0);
            rot.set(0,0,0,1);
            scale.set(1,1,1);

            if (!node.translation.empty())
            {
                pos.set(node.translation[0], node.translation[1], node.translation[2]);
            }
            if (!node.rotation.empty())
            {
                rot.set(node.rotation[0], node.rotation[1], node.rotation[2], node.rotation[3]);
            }
            if (!node.scale.empty())
            {
                scale.set(node.scale[0], node.scale[1], node.scale[2]);
            }

            T.loadIdentity();
            T.translate(pos);
            R.loadIdentity();
            R.toMatrix(rot);
            S.loadIdentity();
            S.scale(scale);
            bone.axis_base = T*R*S;
        }

        lst_bones.push_back(bone);

        if (p_info_bone)
        {
            std::string str;

            ECO_error_set("");
            ECO_error_set("name %s > %d / part %d", bone.name.c_str(), i_nod, bone.index_part);
            ECO_error_set("pos : %f %f %f", pos.x, pos.y, pos.z);
            ECO_error_set("rot : %f %f %f %f", rot.x, rot.y, rot.z, rot.w);
            ECO_error_set("sca : %f %f %f", scale.x, scale.y, scale.z);
            str = "  enfants: ";
            for (const auto& child : bone.children)
            {
                str = str + ECO_intToStr(child) + " ";
            }
            ECO_error_set(str.c_str());
        }
    // end for i_nod
    }

    // recupere les parents
    for (size_t i_bon = 0; i_bon < lst_bones.size(); i_bon++)
    {
        for (size_t i_chi = 0; i_chi < lst_bones[i_bon].children.size(); i_chi++)
        {
            lst_bones[lst_bones[i_bon].children[i_chi]].parent = i_bon;
        }
    }
    // le root pour les animations par bone / *** es ce que cela fonctionne toujours ?
    for (size_t i_bon = 0; i_bon < lst_bones.size(); i_bon++)
    {
        if (lst_bones[i_bon].parent == -1 && i_bon != (size_t)m_root_joint)
        {
            m_root_bone = i_bon;
            //ECO_error_set("bone %d", i_bon);
            break;
        }
    }

    return true;
}

bool OMesh_gltf::_load_material(tinygltf::Model& model, std::string& pathfile)
{
    std::vector<std::string> lstImages;
    std::string path, path_old;
    OSurface surface;

    path_old = OTextureList::P_getPath();
    path = ECO_fileGetPath(pathfile);
    OTextureList::P_setPath(path);
    OTextureList::P_setFilter(p_filter);

    lstImages.reserve(model.images.size());

    for (size_t i_ima = 0; i_ima < model.images.size(); i_ima++)
    {
        const auto &image = model.images[i_ima];

        size_t idx = lstImages.size();
        lstImages.push_back(std::string());

        //ECO_error_set("image %s", model.images[i_ima].name.c_str());
        //ECO_error_set("uri %s", model.images[i_ima].uri.c_str());
        //ECO_error_set("mimeType %s", model.images[i_ima].mimeType.c_str());

        if (image.uri.empty())
        {
            if (!image.image.empty())
            {
                // il est des images sans reference de nom, ce qui pose probleme dans notre cas
                // les texture ayant un nom identique n'étant chargées qu'une fois
                lstImages[idx] = m_file + ECO_valToStr(i_ima);
                surface.create(image.width, image.height, image.bits, GL_RGBA, image.image);
                texturelist.add( lstImages[idx], surface, false); // false = no flipV l'image = laisse-là à l'endroit
            }
            else
            {
                ECO_error_set("erreur image.image %s >OMesh_gltf.load_material", m_file.c_str());
            }
        }
        else
        {
            lstImages[idx] = model.images[i_ima].uri;
            surface.create(image.width, image.height, image.bits, GL_RGBA, image.image);
            texturelist.add( lstImages[idx], false); // false = no flipV l'image = laisse-là à l'endroit
        }
        //ECO_error_set("ref %s", lstImages[idx].c_str());
    }
    // *** probleme sur certaines images internes ? pourquoi
    OTextureList::P_setPath(path_old);

    // --- construction des materials

    for (size_t i_mat = 0; i_mat < model.materials.size(); i_mat++)
    {
        const tinygltf::Material &gltf_mat = model.materials[i_mat];
        int i_tex;

        material.push_back(s_gltf_material());

        // --- baseColor

        i_tex = gltf_mat.pbrMetallicRoughness.baseColorTexture.index;
        //ECO_error_set("baseColor index %d", i_tex);
        if (i_tex > -1)
        {
            if (i_tex >= (int)lstImages.size()) i_tex = 0; // corrige les materials surnuméraires

            material[i_mat].map_diffuse = lstImages[i_tex];
            material[i_mat].tex_diffuse = texturelist.getID(lstImages[i_tex]);
            //ECO_error_set("baseColor image %d = %s", i_tex, lstImages[i_tex].c_str());
        }
        else
        {
            float r,g,b,a;

            r = gltf_mat.pbrMetallicRoughness.baseColorFactor[0];
            g = gltf_mat.pbrMetallicRoughness.baseColorFactor[1];
            b = gltf_mat.pbrMetallicRoughness.baseColorFactor[2];
            a = gltf_mat.pbrMetallicRoughness.baseColorFactor[3];
            material[i_mat].baseColor.set(r,g,b,a);

            // idem phong obj
            r = sqrt( r * 100.0f) * 0.1f;
            g = sqrt( g * 100.0f) * 0.1f;
            b = sqrt( b * 100.0f) * 0.1f;
            a = sqrt( a * 100.0f) * 0.1f;
            material[i_mat].diffuseColor.set(r,g,b,a);

            texturelist.add(gltf_mat.name, r,g,b,a);
            material[i_mat].tex_diffuse = texturelist.getID(gltf_mat.name);
            //ECO_error_set("materiel %d = %s > %f %f %f %f", i_mat, gltf_mat.name.c_str(), r,g,b,a);
        }

        // --- metallicRoughness

        i_tex = gltf_mat.pbrMetallicRoughness.metallicRoughnessTexture.index;
        //ECO_error_set("specular index %d", i_tex);
        if (i_tex > -1)
        {
            if (i_tex >= (int)lstImages.size()) i_tex = 0; // corrige les materials surnuméraires

            material[i_mat].map_specular = lstImages[i_tex];
            material[i_mat].tex_specular = texturelist.getID(lstImages[i_tex]);
            //ECO_error_set("specular image %d = %s", i_tex, image[i_tex].c_str());
        }
        else
        {
            float r,g,b,a;

            r = 0.5f; //gltf_mat.pbrMetallicRoughness.baseColorFactor[0];
            g = 0.5f; //gltf_mat.pbrMetallicRoughness.baseColorFactor[1];
            b = 0.5f; //gltf_mat.pbrMetallicRoughness.baseColorFactor[2];
            a = 1.0f; //gltf_mat.pbrMetallicRoughness.baseColorFactor[3];
            material[i_mat].specular.set(r,g,b,a);

            std::string name = material[i_mat].name + "_s";
            texturelist.add(name, r,g,b,a);
            material[i_mat].tex_specular = texturelist.getID(name);
            //ECO_error_set("materiel %d = %s > %f %f %f %f", i_mat, name.c_str(), r,g,b,a);
        }
        material[i_mat].specular_factor = gltf_mat.pbrMetallicRoughness.metallicFactor;
        material[i_mat].roughness_factor = gltf_mat.pbrMetallicRoughness.roughnessFactor;
        //ECO_error_set("%f %f", material[i_mat].specular_factor, material[i_mat].roughness_factor);

        std::string name = "roughness_map";
        texturelist.add(name, material[i_mat].roughness_factor, 1,1,1); // red prit en compte
        material[i_mat].tex_roughness = texturelist.getID(name);

        // --- emissive

        i_tex = gltf_mat.emissiveTexture.index;
        //ECO_error_set("emissive index %d", i_tex);
        if (i_tex > -1)
        {
            if (i_tex >= (int)lstImages.size()) i_tex = 0; // corrige les materials surnuméraires

            material[i_mat].map_emissive = lstImages[i_tex];
            material[i_mat].tex_emissive = texturelist.getID(lstImages[i_tex]);
            //ECO_error_set("emissive image %d = %s", i_tex, lstImages[i_tex].c_str());
            material[i_mat].emissive.set(0,0,0,1); // avec texture il faut une valeur d'energie
        }
        else
        {
            material[i_mat].emissive.set(0,0,0,0);
            if (gltf_mat.emissiveFactor[0] + gltf_mat.emissiveFactor[1] + gltf_mat.emissiveFactor[2] > 0.0f)
            {
                material[i_mat].emissive.r = gltf_mat.emissiveFactor[0];
                material[i_mat].emissive.g = gltf_mat.emissiveFactor[1];
                material[i_mat].emissive.b = gltf_mat.emissiveFactor[2];
                material[i_mat].emissive.a = 1.0f;

                std::string name = gltf_mat.name + "_e";
                texturelist.add(name, material[i_mat].emissive.r, material[i_mat].emissive.g, material[i_mat].emissive.b, 1.0f);
                material[i_mat].tex_emissive = texturelist.getID(name);
                //ECO_error_set("ok %s", name.c_str());
                //ECO_error_set("%f %f %f", material[i_mat].emissive.r, material[i_mat].emissive.g, material[i_mat].emissive.b);
            }
        }

        // --- normal

        i_tex = gltf_mat.normalTexture.index;
        //ECO_error_set("normal index %d", i_tex);
        if (i_tex > -1)
        {
            if (i_tex >= (int)lstImages.size()) i_tex = 0; // corrige les materials surnuméraires

            material[i_mat].map_normal = lstImages[i_tex];
            material[i_mat].tex_normal = texturelist.getID(lstImages[i_tex]);
            //ECO_error_set("normal image %d = %s", i_tex, lstImages[i_tex].c_str());
        }
        else
        {
            std::string name = "normal_map";
            texturelist.add(name, 0.5f, 0.5f, 1.0f, 1.0f); // ! attention à la valeur alpha = 1.0 !
            material[i_mat].tex_normal = texturelist.getID(name);
        }

        // --- occlusion

        i_tex = gltf_mat.occlusionTexture.index;
        if (i_tex > -1)
        {
            if (i_tex >= (int)lstImages.size()) i_tex = 0; // corrige les materials surnuméraires

            material[i_mat].map_occlusion = lstImages[i_tex];
            material[i_mat].tex_occlusion = texturelist.getID(lstImages[i_tex]);
            //ECO_error_set("occlusion image %d = %s", i_tex, lstImages[i_tex].c_str());
        }
        else
        {
            std::string name = "occlusion_map";
            texturelist.add(name, 1,1,1,1); // red prit en compte
            material[i_mat].tex_occlusion = texturelist.getID(name);
            //ECO_error_set("occlusion image %s %d", name.c_str(), material[i_mat].tex_occlusion );
        }
    //end for
    }

    return true;
}

bool OMesh_gltf::_load_morphTargets(tinygltf::Model &model)
{
    size_t i_par = 0;

    if (p_info_morph) ECO_error_set("--- Mesh: %s", m_file.c_str());

    for (const auto &mesh : model.meshes)
    {
        for (const auto &primitive : mesh.primitives)
        {
            if (primitive.targets.empty()) continue;
            //if (p_info_morph) ECO_error_set("  Morph Targets: %zu", primitive.targets.size());

            for (size_t i_tgt = 0; i_tgt < primitive.targets.size(); i_tgt++)
            {
                std::vector<OVec3_f> morphPos;
                std::vector<OVec3_f> morphNorm;
                std::vector<OVec4_f> morphTan;

                for (const auto &attr : primitive.targets[i_tgt])
                {
                    const std::string attribName = attr.first;
                    int accessorIndex = attr.second;

                    // Debug complet
                    //if (p_info_morph) _logAccessorInfo(model, accessorIndex, attribName);

                    if (attribName == "POSITION")
                        morphPos = _readAccessorVec3(model, accessorIndex);
                    else if (attribName == "NORMAL")
                        morphNorm = _readAccessorVec3(model, accessorIndex);
                    else if (attribName == "TANGENT")
                        morphTan = _readAccessorVec4(model, accessorIndex);
                }

                // Vérification cohérence POSITION / NORMAL
                if (!morphNorm.empty() && morphNorm.size() != morphPos.size())
                {
                    _setError(ERR_MORPH0, i_tgt, i_par, morphPos.size(), morphNorm.size());
                    // Pour éviter les crashs : tronquer NORMAL à la taille de POSITION
                    if (morphNorm.size() > morphPos.size())
                        morphNorm.resize(morphPos.size());
                    else
                        morphNorm.clear(); // On drop les normales si elles sont incomplètes
                }

                // Sauvegarde des données
                meshPart[i_par].morph_positions.push_back(morphPos);
                if (!morphNorm.empty()) meshPart[i_par].morph_normals.push_back(morphNorm);
                if (!morphTan.empty()) meshPart[i_par].morph_tangeants.push_back(morphTan);
                meshPart[i_par].morph_weights.push_back(0.0f);
            }
        }

        if (p_info_morph)
        {
            ECO_error_set("%s", meshPart[i_par].name.c_str());
            ECO_error_set("morph pos : %d", meshPart[i_par].morph_positions.size());
            ECO_error_set("morph nor : %d", meshPart[i_par].morph_normals.size());
            ECO_error_set("morph tan : %d", meshPart[i_par].morph_tangeants.size());
            ECO_error_set("morph wei : %d", meshPart[i_par].morph_weights.size());
        }

        i_par++;
    }

    return true;
}

std::vector<OVec3_f> OMesh_gltf::_readAccessorVec3(const tinygltf::Model &model, int accessorIndex)
{
    std::vector<OVec3_f> out;

    if (accessorIndex < 0 || accessorIndex >= static_cast<int>(model.accessors.size()))
    {
        _setError(ERR_MORPH1, accessorIndex);
        return out;
    }

    const tinygltf::Accessor &accessor = model.accessors[accessorIndex];
    if (accessor.type != TINYGLTF_TYPE_VEC3 || accessor.componentType != TINYGLTF_COMPONENT_TYPE_FLOAT)
    {
        _setError(ERR_MORPH2, accessorIndex);
        return out;
    }

    if (accessor.count == 0) return out;

    // --- Cas SPARSE ---
    if (accessor.sparse.isSparse)
    {
        out.assign(accessor.count, OVec3_f(0.f, 0.f, 0.f)); // valeurs par défaut (deltas)

        const tinygltf::Accessor::Sparse &sparse = accessor.sparse;

        // Récup indices
        const tinygltf::BufferView &indicesView = model.bufferViews[sparse.indices.bufferView];
        const tinygltf::Buffer &indicesBuffer = model.buffers[indicesView.buffer];
        const unsigned char *indicesPtr = indicesBuffer.data.data() + indicesView.byteOffset + sparse.indices.byteOffset;

        // Récup valeurs
        const tinygltf::BufferView &valuesView = model.bufferViews[sparse.values.bufferView];
        const tinygltf::Buffer &valuesBuffer = model.buffers[valuesView.buffer];
        const unsigned char *valuesPtr = valuesBuffer.data.data() + valuesView.byteOffset + sparse.values.byteOffset;

        for (int i = 0; i < sparse.count; i++)
        {
            int index = -1;
            // indices = UNSIGNED_BYTE / SHORT / INT
            switch (sparse.indices.componentType)
            {
            case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:
                index = *(reinterpret_cast<const uint8_t*>(indicesPtr + i * sizeof(uint8_t)));
                break;
            case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT:
                index = *(reinterpret_cast<const uint16_t*>(indicesPtr + i * sizeof(uint16_t)));
                break;
            case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT:
                index = *(reinterpret_cast<const uint32_t*>(indicesPtr + i * sizeof(uint32_t)));
                break;
            default:
                _setError(ERR_MORPH3);
                return out;
            }

            const float *f = reinterpret_cast<const float *>(valuesPtr + i * sizeof(float) * 3);
            if (index >= 0 && index < static_cast<int>(accessor.count)) out[index] = OVec3_f(f[0], f[1], f[2]);
        }

        return out;
    }

    // --- Cas normal avec bufferView ---
    if (accessor.bufferView < 0 || accessor.bufferView >= static_cast<int>(model.bufferViews.size()))
    {
        _setError(ERR_MORPH4, accessorIndex);
        return out;
    }

    const tinygltf::BufferView &bufferView = model.bufferViews[accessor.bufferView];
    if (bufferView.buffer < 0 || bufferView.buffer >= static_cast<int>(model.buffers.size()))
    {
        _setError(ERR_MORPH5, accessor.bufferView);
        return out;
    }

    const tinygltf::Buffer &buffer = model.buffers[bufferView.buffer];

    int stride = accessor.ByteStride(bufferView);
    if (stride <= 0) stride = sizeof(float) * 3;

    size_t neededBytes = accessor.count * stride;
    size_t availableBytes = bufferView.byteLength;
    size_t accessor_count = accessor.count;

    if (neededBytes > availableBytes)
    {
        size_t maxCount = availableBytes / stride;
        _setError(ERR_MORPH6, accessorIndex, accessor.count, maxCount);
        accessor_count = maxCount;
    }

    const unsigned char *dataPtr = buffer.data.data()
        + bufferView.byteOffset
        + accessor.byteOffset;

    out.reserve(accessor_count);
    for (size_t i = 0; i < accessor_count; i++)
    {
        const float *f = reinterpret_cast<const float *>(dataPtr + i * stride);
        out.emplace_back(f[0], f[1], f[2]);
    }

    return out;
}

std::vector<OVec4_f> OMesh_gltf::_readAccessorVec4(const tinygltf::Model &model, int accessorIndex)
{
    std::vector<OVec4_f> out;

    if (accessorIndex < 0 || accessorIndex >= static_cast<int>(model.accessors.size()))
    {
        _setError(ERR_MORPH10, accessorIndex);
        return out;
    }

    const tinygltf::Accessor &accessor = model.accessors[accessorIndex];
    if (accessor.type != TINYGLTF_TYPE_VEC3 || accessor.componentType != TINYGLTF_COMPONENT_TYPE_FLOAT)
    {
        _setError(ERR_MORPH11, accessorIndex);
        return out;
    }

    if (accessor.count == 0) return out;

    // --- Cas SPARSE ---
    if (accessor.sparse.isSparse)
    {
        out.assign(accessor.count, OVec4_f(0,0,0,0)); // valeurs par défaut (deltas)

        const tinygltf::Accessor::Sparse &sparse = accessor.sparse;

        // Récup indices
        const tinygltf::BufferView &indicesView = model.bufferViews[sparse.indices.bufferView];
        const tinygltf::Buffer &indicesBuffer = model.buffers[indicesView.buffer];
        const unsigned char *indicesPtr = indicesBuffer.data.data() + indicesView.byteOffset + sparse.indices.byteOffset;

        // Récup valeurs
        const tinygltf::BufferView &valuesView = model.bufferViews[sparse.values.bufferView];
        const tinygltf::Buffer &valuesBuffer = model.buffers[valuesView.buffer];
        const unsigned char *valuesPtr = valuesBuffer.data.data() + valuesView.byteOffset + sparse.values.byteOffset;

        for (int i = 0; i < sparse.count; i++)
        {
            int index = -1;
            // indices = UNSIGNED_BYTE / SHORT / INT
            switch (sparse.indices.componentType)
            {
            case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:
                index = *(reinterpret_cast<const uint8_t*>(indicesPtr + i * sizeof(uint8_t)));
                break;
            case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT:
                index = *(reinterpret_cast<const uint16_t*>(indicesPtr + i * sizeof(uint16_t)));
                break;
            case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT:
                index = *(reinterpret_cast<const uint32_t*>(indicesPtr + i * sizeof(uint32_t)));
                break;
            default:
                _setError(ERR_MORPH12);
                return out;
            }

            const float *f = reinterpret_cast<const float *>(valuesPtr + i * sizeof(float) * 4);
            if (index >= 0 && index < static_cast<int>(accessor.count)) out[index] = OVec4_f(f[0], f[1], f[2], f[3]);
        }

        return out;
    }

    // --- Cas normal avec bufferView ---
    if (accessor.bufferView < 0 || accessor.bufferView >= static_cast<int>(model.bufferViews.size()))
    {
        _setError(ERR_MORPH13, accessorIndex);
        return out;
    }

    const tinygltf::BufferView &bufferView = model.bufferViews[accessor.bufferView];
    if (bufferView.buffer < 0 || bufferView.buffer >= static_cast<int>(model.buffers.size()))
    {
        _setError(ERR_MORPH14, accessor.bufferView);
        return out;
    }

    const tinygltf::Buffer &buffer = model.buffers[bufferView.buffer];

    int stride = accessor.ByteStride(bufferView);
    if (stride <= 0) stride = sizeof(float) * 4;

    size_t neededBytes = accessor.count * stride;
    size_t availableBytes = bufferView.byteLength;
    size_t accessor_count = accessor.count;

    if (neededBytes > availableBytes)
    {
        size_t maxCount = availableBytes / stride;
        _setError(ERR_MORPH15, accessorIndex, accessor.count, maxCount);
        accessor_count = maxCount;
    }

    const unsigned char *dataPtr = buffer.data.data()
        + bufferView.byteOffset
        + accessor.byteOffset;

    out.reserve(accessor_count);
    for (size_t i = 0; i < accessor_count; i++)
    {
        const float *f = reinterpret_cast<const float *>(dataPtr + i * stride);
        out.emplace_back(f[0], f[1], f[2], f[3]);
    }

    return out;
}

// debug
void OMesh_gltf::_logAccessorInfo(const tinygltf::Model &model, int accessorIndex, const std::string& label)
{
    if (accessorIndex < 0 || accessorIndex >= static_cast<int>(model.accessors.size()))
    {
        ECO_error_set("[Accessor %d] %s -> INVALID index", accessorIndex, label.c_str());
        return;
    }

    const tinygltf::Accessor &acc = model.accessors[accessorIndex];
    ECO_error_set("[Accessor %d] %s", accessorIndex, label.c_str());
    ECO_error_set("  count=%d, type=%d, compType=%d, normalized=%d", (int)acc.count, acc.type, acc.componentType, acc.normalized);
    ECO_error_set("  bufferView=%d, byteOffset=%d", acc.bufferView, (int)acc.byteOffset);

    if (acc.bufferView >= 0 && acc.bufferView < static_cast<int>(model.bufferViews.size()))
    {
        const tinygltf::BufferView &bv = model.bufferViews[acc.bufferView];
        ECO_error_set("  BufferView: buffer=%d, byteOffset=%d, byteLength=%d, byteStride=%d", bv.buffer, (int)bv.byteOffset, (int)bv.byteLength, (int)bv.byteStride);

        if (bv.buffer >= 0 && bv.buffer < static_cast<int>(model.buffers.size()))
        {
            const tinygltf::Buffer &buf = model.buffers[bv.buffer];
            ECO_error_set("  Buffer size=%d bytes", (int)buf.data.size());
        }
    }
    else
    {
        ECO_error_set("  (no valid bufferView)");
    }
}

bool OMesh_gltf::_load_skin(tinygltf::Model &model)
{
    std::string str;

    if (p_info_bone)
    {
        ECO_error_set(" ");
        ECO_error_set(m_file.c_str());
        ECO_error_set(">joints>");
    }

    // i_ski = null ou 0 le plus souvent > attention break en fin de boucle
    for (size_t i_ski = 0; i_ski < model.skins.size(); i_ski++)
    {
        const tinygltf::Skin &skin = model.skins[i_ski];

        lst_joints.reserve(skin.joints.size());

        for (size_t i_joi = 0; i_joi < skin.joints.size(); i_joi++)
        {
            int jointNodeIndex = skin.joints[i_joi];
            const tinygltf::Node &node = model.nodes[jointNodeIndex];

            s_gltf_joint joint;

            joint.name = node.name;
            joint.index_bone = jointNodeIndex;
                lst_bones[jointNodeIndex].index_joint = i_joi; // pour l'animation
            joint.children_bone = node.children; // attention : children bone 22 = joint 1 ! il faut rechercher les enfants !
            joint.axis_computed.loadIdentity();
            joint.axis_inverse.loadIdentity();

            lst_joints.push_back(joint);

            if (p_info_bone)
            {
                ECO_error_set("joint %d = bone %d", i_joi, jointNodeIndex);
                for (size_t i = 0; i < lst_joints[i_joi].children_bone.size(); i++)
                {
                    ECO_error_set("      > %d",  lst_joints[i_joi].children_bone[i] );
                }
            }
        // end skin joint
        }

        if (skin.inverseBindMatrices >= 0)
        {
            const tinygltf::Accessor &accessor = model.accessors[skin.inverseBindMatrices];
            const tinygltf::BufferView &bufferView = model.bufferViews[accessor.bufferView];
            const tinygltf::Buffer &buffer = model.buffers[bufferView.buffer];

            const float *data = reinterpret_cast<const float*>(&buffer.data[bufferView.byteOffset + accessor.byteOffset]);
            for (size_t i = 0; i < accessor.count; ++i)
            {
                for (size_t j = 0; j < 16; j++)
                {
                    lst_joints[i].axis_inverse[j] = data[(i*16) + j];
                }
                lst_joints[i].axis_inverse.transpose(); // si mat 'horizontale' = de mat col vers mat row
            }
        }
        else
        {
            for (size_t i_joi = 0; i_joi < lst_joints.size(); i_joi++)
            {
                lst_joints[i_joi].axis_inverse = lst_bones[lst_joints[i_joi].index_bone].axis_base;
                lst_joints[i_joi].axis_inverse.transpose();
            }
        }
    //end skin
    }

    // recherche des index enfants des joints
    for (size_t i_joi = 0; i_joi < lst_joints.size(); i_joi++)
    {
        for (size_t i_chi = 0; i_chi < lst_joints[i_joi].children_bone.size(); i_chi++)
        {
            int idx = lst_joints[i_joi].children_bone[i_chi];
            for (size_t i = 0; i < lst_joints.size(); i++)
            {
                if (lst_joints[i].index_bone == idx)
                {
                    lst_joints[i_joi].children_joint.push_back(i);
                    break;
                }
            }
        }
    }

    return true;
}







void OMesh_gltf::setAmim_neutral_auto(bool mode)
{
    m_anim_neutral = mode;
}

// #include <stdarg.h>	//argument des erreurs
void OMesh_gltf::_setError(int idx, ...)
{
    const char* format = gltf_err[idx].c_str();
    char* msg = new char[512];
    if (list_error.size() == 0)
    {
        list_error.push_back(m_path + " " + m_file);
    }
    va_list argList;
    va_start(argList, idx);
    vsnprintf(msg, 512, format, argList);
    va_end(argList);
    strcat(msg, "\n");

    list_error.push_back(msg);
}

void OMesh_gltf::setModelMulti(std::vector<OMat4>& modelMulti)
{
	m_num_instances = modelMulti.size();

    for (auto& part : meshPart)
    {
        glBindVertexArray(part.vao);
        glBindBuffer(GL_ARRAY_BUFFER, part.vao_buffer[LOCATION_MODEL]);
        glBufferData(GL_ARRAY_BUFFER, sizeof(OMat4) * m_num_instances, &modelMulti[0], GL_DYNAMIC_DRAW);
    }

    //désactivation du VAO
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void OMesh_gltf::setPart_visibility(size_t part, bool mode)
{
    meshPart[part].visibility = mode;
}

void OMesh_gltf::setPart_visibility(std::string ref_part, bool mode)
{
    for (auto& part : meshPart)
    {
        if (part.name == ref_part)
        {
            part.visibility = mode;
            break;
        }
    }
}


// necessite openGL 4.4 (glBufferStorage)
// voir OMesh_gltf old dans le dossier ECO/
void OMesh_gltf::_make_vao()
{
    //int gltf_nbr_vao_buffer = 10; > correspond au shader !

    for (auto& part : meshPart)
    {
        // Génération de l'identifiant du VAO
        glGenVertexArrays(1, &part.vao);
        glBindVertexArray(part.vao);

        // Création des buffers
        glGenBuffers(gltf_nbr_vao_buffer, part.vao_buffer);

        // --- Buffer pour les positions (dynamique) ---
        glBindBuffer(GL_ARRAY_BUFFER, part.vao_buffer[LOCATION_POSITION]);
        // Alloue le buffer avec la taille maximale et le marque comme "persistant"
        glBufferStorage(
            GL_ARRAY_BUFFER,
            sizeof(OVec3_f) * part.vertex.capacity(),  // Capacité maximale
            nullptr,  // Pas de données initiales
            GL_MAP_WRITE_BIT | GL_MAP_PERSISTENT_BIT | GL_MAP_COHERENT_BIT
        );
        // Mappe le buffer pour un accès persistant
        void* persistentPosBuffer = glMapBufferRange(
            GL_ARRAY_BUFFER,
            0,
            sizeof(OVec3_f) * part.vertex.capacity(),
            GL_MAP_WRITE_BIT | GL_MAP_PERSISTENT_BIT | GL_MAP_COHERENT_BIT
        );
        // Copie les données initiales
        if (persistentPosBuffer && !part.vertex.empty()) {
            memcpy(persistentPosBuffer, part.vertex.data(), sizeof(OVec3_f) * part.vertex.size());
        }
        glEnableVertexAttribArray(LOCATION_POSITION);
        glVertexAttribPointer(LOCATION_POSITION, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));

        // --- Buffer pour les normales (dynamique) ---
        glBindBuffer(GL_ARRAY_BUFFER, part.vao_buffer[LOCATION_NORMAL]);
        glBufferStorage(
            GL_ARRAY_BUFFER,
            sizeof(OVec3_f) * part.normal.capacity(),
            nullptr,
            GL_MAP_WRITE_BIT | GL_MAP_PERSISTENT_BIT | GL_MAP_COHERENT_BIT
        );
        void* persistentNormBuffer = glMapBufferRange(
            GL_ARRAY_BUFFER,
            0,
            sizeof(OVec3_f) * part.normal.capacity(),
            GL_MAP_WRITE_BIT | GL_MAP_PERSISTENT_BIT | GL_MAP_COHERENT_BIT
        );
        if (persistentNormBuffer && !part.normal.empty()) {
            memcpy(persistentNormBuffer, part.normal.data(), sizeof(OVec3_f) * part.normal.size());
        }
        glEnableVertexAttribArray(LOCATION_NORMAL);
        glVertexAttribPointer(LOCATION_NORMAL, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));

        // --- Buffer pour les UVs (statique) ---
        glBindBuffer(GL_ARRAY_BUFFER, part.vao_buffer[LOCATION_TEXCOORD]);
        glBufferData(
            GL_ARRAY_BUFFER,
            sizeof(OVec2_f) * part.uv.size(),
            part.uv.data(),
            GL_STATIC_DRAW
        );
        glEnableVertexAttribArray(LOCATION_TEXCOORD);
        glVertexAttribPointer(LOCATION_TEXCOORD, 2, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));

        // --- Buffer pour les matrices de modèle (instanciation) ---
        glBindBuffer(GL_ARRAY_BUFFER, part.vao_buffer[LOCATION_MODEL]);
        glBufferData(GL_ARRAY_BUFFER, sizeof(OMat4) * m_num_instances, nullptr, GL_DYNAMIC_DRAW);
        for (int i = 0; i < 4; i++) {
            glEnableVertexAttribArray(LOCATION_MODEL + i);
            glVertexAttribPointer(LOCATION_MODEL + i, 4, GL_FLOAT, GL_FALSE, sizeof(OMat4), (const void*)(sizeof(float) * i * 4));
            glVertexAttribDivisor(LOCATION_MODEL + i, 1);
        }

        // --- Buffer pour les indices des joints (statique) ---
        glBindBuffer(GL_ARRAY_BUFFER, part.vao_buffer[LOCATION_BONE]);
        glBufferData(
            GL_ARRAY_BUFFER,
            sizeof(OVec4_ui) * part.jointWeight.size(),
            part.jointWeight.data(),
            GL_STATIC_DRAW
        );
        glEnableVertexAttribArray(LOCATION_BONE);
        glVertexAttribIPointer(LOCATION_BONE, 4, GL_UNSIGNED_INT, 0, BUFFER_OFFSET(0));

        // --- Buffer pour les poids des joints (statique) ---
        glBindBuffer(GL_ARRAY_BUFFER, part.vao_buffer[LOCATION_WEIGHT]);
        glBufferData(
            GL_ARRAY_BUFFER,
            sizeof(OVec4_f) * part.weight.size(),
            part.weight.data(),
            GL_STATIC_DRAW
        );
        glEnableVertexAttribArray(LOCATION_WEIGHT);
        glVertexAttribPointer(LOCATION_WEIGHT, 4, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));

        // --- Buffer pour les indices (statique) ---
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, part.vao_buffer[9]);
        glBufferData(
            GL_ELEMENT_ARRAY_BUFFER,
            sizeof(unsigned int) * part.indice.size(),
            part.indice.data(),
            GL_STATIC_DRAW
        );

        // --- Stocke les pointeurs persistants pour mise à jour ultérieure ---
        part.persistentPosBuffer = persistentPosBuffer;
        part.persistentNormBuffer = persistentNormBuffer;
    }

    // Désactive les buffers/VAO
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}


// ----------------------
// --- les propriétés ---
// ----------------------

void OMesh_gltf::P_setPath(std::string path)
{
    p_path = path;
}

void OMesh_gltf::P_setMipmap(bool mode)
{
    p_mipmap = mode;
}

void OMesh_gltf::P_setScale(float x, float y, float z)
{
    p_scale.set(x,y,z);
}

// efface la texture / decompte si elle est utilisée plusieurs fois = efface si zéro
void OMesh_gltf::P_clear_texture(std::vector<s_gltf_material>& material)
{
    for (auto& mat : material)
    {
        texturelist.del( mat.tex_diffuse );
        texturelist.del( mat.tex_normal );
        texturelist.del( mat.tex_specular );
        texturelist.del( mat.tex_emissive );
        texturelist.del( mat.tex_occlusion );
        texturelist.del( mat.tex_roughness );
    }
}

void OMesh_gltf::P_setFilter(int filter)
{
    p_filter = filter;
}

void OMesh_gltf::P_setInfoBones(bool mode)
{
    p_info_bone = mode;
}

void OMesh_gltf::P_setInfoAnim(bool mode)
{
    p_info_anim = mode;
}

void OMesh_gltf::P_setInfoHierarchy(bool mode)
{
    p_info_hierarchy = mode;
}

void OMesh_gltf::P_setInfoMorph(bool mode)
{
    p_info_morph = mode;
}






