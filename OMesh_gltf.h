
#ifndef OMESH_GLTF_H_INCLUDE
#define OMESH_GLTF_H_INCLUDE

/*
OBJET       chargeur d'objets 3d glft-glb

DATE        10/2025

AUTEUR      philippe Thebaud

COMMENTAIRE des exemples complets sont disponibles sur https://philthebjob.wixsite.com/moteur3d-eco

            uniquement face triangle.
            je charge les meshs comme autant de parties : 2 persos de 6 parties (tete, corps, bras, jambes) font 12 meshPart.
            ce petit chargeur est destiné à des meshs simples : perso, objet, ...

            texture externes ou internes, ne charge qu'une fois la texture avec référence identique (même source image)
                albedo > normal > metallic > roughness > ambient occlusion > emissive

            les objets associés (épée d'un guerrier par exemple) sont pris en compte > GLTF_ANIM_AXIS

            Les animations sans joint (cas de chest_axis.glb de otl Photo), ne sont pas pris en compte de la manière usuelle :
            ici, j'ajoute des joints, ce qui permet d'utiliser simplement anim_getAxis().

            les animations par bone (cas du dragon de otl photo) sont pris en compte  > GLTF_ANIM_BONE

            les transformations ne le sont pas :
            > définissez le point d'origine (Blender > point orangé)
            > appliquez toutes les transformations (Blender > menu Objet>Appliquer>Toutes...)
            > nettoyez les données (Blender > menu Fichier>Nettoyer>Bloc...)

            ma matrice est 'horizontale' > supprimez les lignes 'mat.transpose' si votre matrice est 'verticale'
            ex : lst_joints[i_bon].axisInverse.transpose(); // ma matrice 'horizontale'

            les morph target correspondent à des capes ou des drapeaux : ils sont lus à la volé lors de l'animation,
            puis calculés lors de l'affichage. Ils ne nécessitent pas de shader spécifique. Pour aller plus loin, il faut
            utiliser des vbo de stockage, précalculer l'affichage et utiliser des shaders specifiques.

            Il est parfois utile de charger le mesh avec la visionneuse de Windows (pas depuis Blender), et de le sauver :
            cela 'renormalise' le mesh (je n'ai pas traité tous les cas de mesh 'tordus').

            utilisez : (voir Tmesh_gltf de Otl Photo)
            OMesh_gltf::P_setInfoAnim(true);        liste des animations et leur pos rot scale weight
            OMesh_gltf::P_setInfoBones(true);       bones pos rot scale & joint associé
            OMesh_gltf::P_setInfoHierarchy(true);   relations des bones (parent enfants) et des joints
            OMesh_gltf::P_setInfoMorph(true);       liste "d'erreurs" lors de la lecture des morph target
*/

#include <GL/glew.h>
#include <cstring>
#include <vector>
#include <functional>
//#include <set>
#include <unordered_set>
#include <cstdint> // pour les types entiers fixes uint32_t ...
#include <stdarg.h>	//argument de erreurs


#include "OUtils.h"
#include "OTextureList.h"
#include "tiny/tiny_gltf.h"

// ---

struct s_gltf_bone
{
    std::string name;
    std::vector<int> children;
    int         parent;
    int         index_joint;
    int         index_part;
    OMat4       axis_base;          // transform local (depuis glTF)
    // variables pour l'animation
    OVec3_f     pos;
    OVec4_f     rot;
    OVec3_f     scale;
    OMat4       axis_computed;      // transform accumulée (parent → enfant)
    OMat4       axis_trs;

    s_gltf_bone()
    {
        parent = -1;        // -1 = non référencé
        index_joint = -1;
        index_part = -1;
    }
};

struct s_gltf_joint
{
    std::string name;
    std::vector<int> children_bone;
    std::vector<int> children_joint;
    int         index_bone;
    // variables pour l'animation
    OVec3_f     pos;
    OVec4_f     rot;
    OVec3_f     scale;
    OMat4       axis_computed, axis_inverse;

    s_gltf_joint()
    {
        index_bone = -1;
    }
};

struct s_gltf_animationChannel
{
    int         sampler;        //
    int         target_node;    // bone activé
    int         target_meshPart;
    std::string target_path;    // mode (translation rotation scale)
};

struct s_gltf_animationSampler
{
    std::vector<float> time;
    std::vector<float> value;
    int         interpolation;
};
/*
// mode d'animation des parties
enum class GLTF_ANIM_MODE { NULL_ANIM, RIG, AXIS, BONE, MORPH };
// interpolation
enum class GLTF_INTERPOLATION { STEP, LINEAR, CUBICSPLINE, CATMULLROMSPLINE };
*/
// mode d'animation des parties
enum { GLTF_ANIM_NULL, GLTF_ANIM_RIG, GLTF_ANIM_AXIS, GLTF_ANIM_BONE, GLTF_ANIM_MORPH };
static std::string gltf_mode_anim[] = {"null", "rig", "axis", "bone", "morph"};
// interpolation
//const std::string str_interpolation[] = {"STEP", "LINEAR", "CUBICSPLINE", "CATMULLROMSPLINE"};
enum { GLTF_STEP, GLTF_LINEAR, GLTF_CUBICSPLINE, GLTF_CATMULLROMSPLINE };
//


struct s_gltf_animation
{
  std::string name;
  float       duration = 0.0f;
  std::vector<s_gltf_animationChannel> channels;
  std::vector<s_gltf_animationSampler> samplers;
};

struct s_gltf_material
{
    std::string name;
    OVec4_f     baseColor, diffuseColor; // pour pbr et standard
    OVec4_f	    specular;
    OVec4_f     emissive;
    float       opacity;
    float       specular_factor;
    float       roughness_factor;

    // textures (anciens materiaux = diffuse specular -> metallicRoughness (valeurs couleur et brillance) + map...
    std::string	map_diffuse, map_specular, map_normal, map_occlusion, map_emissive, map_roughness;
    uint32_t    tex_diffuse, tex_specular, tex_normal, tex_occlusion, tex_emissive, tex_roughness;

    s_gltf_material() { clear(); }
    void clear()
    {
        name = "";
        baseColor.set(0.8f, 0.8f, 0.8f, 1.0f);
        specular.set(0.0f, 0.0f, 0.0f, 0.25f);
        emissive.set(0,0,0,0); // rgb et factor
        opacity = 1.0f;
        specular_factor = 0.5f;
        roughness_factor = 0.5f;

        //tex_param.clear();
        map_diffuse = ""; map_specular = ""; map_occlusion = ""; map_normal = ""; map_emissive = ""; map_roughness = "";
        tex_diffuse = 0;  tex_specular = 0;  tex_occlusion = 0;  tex_normal = 0;  tex_emissive = 0; tex_roughness = 0;
    }
};

// ---

// le nombre de buffer correspond à l'appel aux shader > location
// ici : vertex normal uv 4*model bone weight et indice
static const int gltf_nbr_vao_buffer = 10;

struct s_gltf_part
{
    bool        visibility;
    std::string name;
    uint32_t    material;
    uint32_t    vao, vao_buffer[gltf_nbr_vao_buffer];
    void*       persistentPosBuffer;
    void*       persistentNormBuffer;

    std::vector<OVec3_f>        vertex;
    std::vector<OVec3_f>        normal;
    std::vector<OVec2_f>        uv;
    //std::vector<uint_fast32_t>  indice;
    std::vector<unsigned int>   indice;
    // bones
    std::vector<OVec4_ui>       jointWeight;
    std::vector<OVec4_f>        weight;
    // morph target > les morph target sont couteux, il vaut mieux éviter de les utiliser.
    // Ici ils sont par defaut en lecture lors de l'animation, ce qui évite de mettre un shader spécifique.
    // Sinon modifiez le vao (+1 buffer par morph !) et le shader (voir exemple dans "otl photo")
    std::vector< std::vector<OVec3_f> > morph_positions;    // [morphIndex][vertexIndex]
    std::vector< std::vector<OVec3_f> > morph_normals;      // idem, optionnel
    std::vector< std::vector<OVec4_f> > morph_tangeants;
    std::vector<float> morph_weights;               // poids courants interpolés
    std::vector<OVec3_f> morph_scratch_positions;   // pour optimisation
    std::vector<OVec3_f> morph_scratch_normals;

    //uint32_t    topologie_style; // = GL_TRIANGLES  (GL_POINT GL_LINE, GL_QUAD GL_POLYGON non codés)
    OVec3_f     bound_min, bound_max;
    int         animation_mode;
    int         joint_ref, bone_ref;
    OMat4       axis_attachment;

    s_gltf_part()
    {
        visibility = true;
        name = "";
        material = 0;
        vao = 0;
        for (size_t i = 0; i < gltf_nbr_vao_buffer; i++) vao_buffer[i] = 0;
        persistentPosBuffer = nullptr;
        persistentNormBuffer = nullptr;

        vertex.clear();
        normal.clear();
        uv.clear();
        indice.clear();
        jointWeight.clear();
        weight.clear();

        morph_positions.clear();
        morph_normals.clear();
        morph_tangeants.clear();
        morph_weights.clear();
        morph_scratch_positions.clear();
        morph_scratch_normals.clear();

        //topologie_style = TOP_TRIANGLE;
        bound_min.set(10000.0f, 10000.0f, 10000.0f);
        bound_max.set(-10000.0f, -10000.0f, -10000.0f);
        animation_mode = GLTF_ANIM_NULL;
        joint_ref = -1, bone_ref = -1;
        axis_attachment.loadIdentity();
    }
};

// --- liste des erreurs/alertes

enum {
    ERR_SCALE,
    //
    ERR_LOAD, ERR_LOAD0, ERR_LOAD1, ERR_LOAD2,
    ERR_MATERIAL, ERR_BONES, ERR_SKIN, ERR_GEOMETRY, ERR_ANIMATIONS, ERR_MORPH, ERR_INIT,
    //
    ERR_MORPH0, ERR_MORPH1, ERR_MORPH2, ERR_MORPH3, ERR_MORPH4, ERR_MORPH5, ERR_MORPH6,
    ERR_MORPH10, ERR_MORPH11, ERR_MORPH12, ERR_MORPH13, ERR_MORPH14, ERR_MORPH15
};

static const std::string gltf_err[] = {
    "Scale error %f : to short",
    //
    "Check loading",
    "GLTF WARNING: %s", "GLTF WARNING: file no found : %s", "GLTF WARNING: failed to load : %s",
    "Check material", "Check bones", "Check skin", "Check geometry", "Check animations", "Check morph", "Check init",
    //
    "WARNING: morph target %d (mesh %d) POSITION(%d) != NORMAL(%d).",
    "invalid accessor index: %d >OMesh_gltf._readAccessorVec3",
    "accessor %d is not VEC3<float> >OMesh_gltf._readAccessorVec3",
    "sparse indices unsupported type >OMesh_gltf._readAccessorVec3",
    "accessor %d has no valid bufferView >OMesh_gltf._readAccessorVec3",
    "bufferView %d has invalid buffer index >OMesh_gltf._readAccessorVec3",
    "truncating accessor %d from %d to %d elements >OMesh_gltf._readAccessorVec3"

    "invalid accessor index: %d >OMesh_gltf._readAccessorVec4",
    "accessor %d is not VEC3<float> >OMesh_gltf._readAccessorVec4",
    "sparse indices unsupported type >OMesh_gltf._readAccessorVec4",
    "accessor %d has no valid bufferView >OMesh_gltf._readAccessorVec4",
    "bufferView %d has invalid buffer index >OMesh_gltf._readAccessorVec4",
    "truncating accessor %d from %d to %d elements >OMesh_gltf._readAccessorVec4"
};

// ---

class OMesh_gltf
{
public:

    OMesh_gltf();
    ~OMesh_gltf();

    bool        anim_getAxis(std::vector<OMat4>& axis);
    bool        anim_morph();
    bool        anim_running();
    bool        anim_start(std::string name, size_t loop = 1);
    bool        anim_start(size_t index, size_t loop = 1);
    void        anim_stop();

    void        clear();
    bool        create(std::string file);

    void        draw(); // pour une texture unique
    void        draw(size_t part);
    void        draw_line(int type = GL_LINES);
    void        draw_line(size_t part, int type = GL_LINES);
    void        draw_multi(size_t part);
    void        draw_multi_line(int type = GL_LINES);
    void        draw_multi_line(size_t part, int type = GL_LINES);
    void        end_draw_part();

    // ---

    int             getAnimation_mode(size_t part) const;
    const std::vector<s_gltf_animation>& getAnimations() const;
    int             getAnimation(std::string ref) const;
    std::string     getAnimation(size_t ref) const;
    float           getAnimation_duration() const;
    size_t          getAnimation_size() const;

    float           getAlpha(size_t part) const;
    const OMat4&    getAxisAttach(size_t idx_joint) const;
    const std::vector<OMat4>& getAxisNeutral() const;
    const OVec4_f&  getBaseColor(size_t part) const;
    bool            getBone(std::string name, s_gltf_bone& result) const;
    const OMat4     getBoneOfPart(size_t part) const;
    const OVec3_f&  getBound_min(int part = -1) const; // -1 =totalité
    const OVec3_f&  getBound_max(int part = -1) const;
    const OVec4_f&  getDiffuse(size_t part) const;
    const OVec4_f&  getEmissive(size_t part) const;

    uint32_t        getId_diffuseMap(size_t part) const;
    uint32_t        getId_emissiveMap(size_t part) const;
    uint32_t        getId_normalMap(size_t part) const;
    uint32_t        getId_occlusionMap(size_t part) const;
    uint32_t        getId_specularMap(size_t part) const;
    uint32_t        getId_albedoMap(size_t part) const;
    uint32_t        getId_metallicMap(size_t part) const;
    uint32_t        getId_roughnessMap(size_t part) const;
    uint32_t        getId_AOMap(size_t part) const;

    bool            getJoint(std::string name, s_gltf_joint& result) const;
    OMat4           getJointOfPart(size_t part) const;

    const s_gltf_material&  getMaterial(size_t part) const;
    size_t          getMaterial_size() const;
    float           getOpacity(size_t part) const; // nouvelle appelation de alpha

    const s_gltf_part& getPart(size_t part) const;
    std::string     getPart_name(size_t part) const;
	size_t          getPart_size() const;
    bool            getPart_visibility(size_t part) const;
    //
	std::string     getPath() const;
    float           getRoughness_factor(size_t part) const;
    float           getScale() const;
    const OVec4_f&  getSpecular(size_t part) const;
    float           getSpecular_factor(size_t part) const;
    size_t          getVerticale() const;

    // ---

    void        setAmim_neutral_auto(bool mode);
    void        setModelMulti(std::vector<OMat4>& modelMulti);
    void        setPart_visibility(size_t part, bool mode);
    void        setPart_visibility(std::string ref_part, bool mode);

    // ---

    static void P_setPath(std::string path);
    static void P_setMipmap(bool mode);
    static void P_setScale(float x, float y, float z);
    static void P_clear_texture(std::vector<s_gltf_material>& material);
    static void P_setFilter(int filter);
    static void P_setInfoAnim(bool mode);   // !!! temps de chargement long
    static void P_setInfoBones(bool mode);
    static void P_setInfoHierarchy(bool mode);
    static void P_setInfoMorph(bool mode);

private:

    void        _getAnim_axis(int idx_parent, int idx_children, std::vector<OMat4>& axis);
    OMat4       _getGlobalTransform(int nodeIndex);
    void        _getNodeHierarchy(const tinygltf::Model &model); //debug

    bool        _init(tinygltf::Model &model);
    void        _init_attach_axis();
    void        _init_attach_bone();

    bool        _load(tinygltf::Model& model, std::string& pathfile);
    bool        _load_animations(tinygltf::Model &model);
    bool        _load_bones(tinygltf::Model& model);
    bool        _load_geometry(tinygltf::Model& model);
    bool        _load_material(tinygltf::Model& model, std::string& pathfile);
    bool        _load_morphTargets(tinygltf::Model& model);
    bool        _load_skin(tinygltf::Model &model);

    // morph target > info et lecture
    void                 _logAccessorInfo(const tinygltf::Model &model, int accessorIndex, const std::string& label = "");
    std::vector<OVec3_f> _readAccessorVec3(const tinygltf::Model &model, int accessorIndex); // lecture des données morph
    std::vector<OVec4_f> _readAccessorVec4(const tinygltf::Model &model, int accessorIndex);

    void        _make_vao();

    void        _setError(int idx, ...);

private:

    std::string m_path, m_file;
    size_t      m_num_instances, m_verticale;
    OVec3_f     m_bound_min, m_bound_max;
    float       m_scale;

    std::vector<s_gltf_bone>        lst_bones;
    int                             m_root_joint, m_root_bone; // root pour les rig/joints & root pour les bone animés
    //
    std::vector<s_gltf_joint>       lst_joints;
    std::vector<s_gltf_animation>   lst_animations;
    //
    std::vector<s_gltf_part>        meshPart;
    std::vector<s_gltf_material>    material;

    int                             m_animation_en_cours;
    uint64_t                        m_start_time;
    int                             m_anim_loop;
    bool                            m_anim_neutral;
    std::vector<OMat4>              lst_neutral_axis;

public:

    // std::string_view
    std::vector<std::string>        list_error;
};

#endif // OMESH_GLTF_H_INCLUDE

