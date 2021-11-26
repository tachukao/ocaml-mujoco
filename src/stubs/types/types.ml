open Ctypes

module Constants (S : Cstubs.Types.TYPE) = struct
  open S

  let mjVERSION_HEADER = constant "mjVERSION_HEADER" int
  let mjFONTSCALE_150 = constant "mjFONTSCALE_150" int
  let mjNAUX = constant "mjNAUX" int
  let mjMAXTEXTURE = constant "mjMAXTEXTURE" int
  let mjMAXOVERLAY = constant "mjMAXOVERLAY" int
  let mjMAXLINE = constant "mjMAXLINE" int
  let mjMAXLINEPNT = constant "mjMAXLINEPNT" int
  let mjMAXPLANEGRID = constant "mjMAXPLANEGRID" int
end

module Bindings (S : Cstubs.Types.TYPE) = struct
  open S
  include Constants (S)

  type mjModel

  let mjModel : mjModel structure typ = structure "mjModel"

  type _mjData

  let _mjData : _mjData structure typ = structure "_mjData"
  let time = field _mjData "time" double
  let () = seal _mjData

  type mjData = _mjData

  let mjData = _mjData

  type mjVFS

  let mjVFS : mjVFS structure typ = structure "mjVFS"

  type _mjvCamera

  let _mjvCamera : _mjvCamera structure typ = structure "_mjvCamera"
  let mjvCamera_type = field _mjvCamera "type" int
  let mjvCamera_fixedcamid = field _mjvCamera "fixedcamid" int
  let mjvCamera_trackbodyid = field _mjvCamera "trackbodyid" int
  let mjvCamera_lookat = field _mjvCamera "lookat" (ptr double)
  let mjvCamera_distance = field _mjvCamera "distance" double
  let mjvCamera_azimuth = field _mjvCamera "azimuth" double
  let mjvCamera_elevation = field _mjvCamera "elevation" double
  let () = seal _mjvCamera

  type mjvCamera = _mjvCamera

  let mjvCamera = _mjvCamera

  type _mjvOption

  let _mjvOption : _mjvOption structure typ = structure "_mjvOption"
  let mjvOption_label = field _mjvOption "label" int
  let mjvOption_frame = field _mjvOption "frame" int
  let mjvOption_geomgroup = field _mjvOption "geomgroup" (ptr uchar)
  let mjvOption_sitegroup = field _mjvOption "sitegroup" (ptr uchar)
  let mjvOption_jointgroup = field _mjvOption "jointgroup" (ptr uchar)
  let mjvOption_tendongroup = field _mjvOption "tendongroup" (ptr uchar)
  let mjvOption_actuatorgroup = field _mjvOption "actuatorgroup" (ptr uchar)
  let mjvOption_flags = field _mjvOption "flags" (ptr uchar)
  let () = seal _mjvOption

  type mjvOption = _mjvOption

  let mjvOption = _mjvOption

  type mjvGeom

  let mjvGeom : mjvGeom structure typ = structure "mjvGeom"

  type mjvLight

  let mjvLight : mjvLight structure typ = structure "mjvLight"

  type mjvGLCamera

  let mjvGLCamera : mjvGLCamera structure typ = structure "mjvGLCamera"

  type _mjvScene

  let _mjvScene : _mjvScene structure typ = structure "_mjvScene"
  let mjvScene_maxgeom = field _mjvScene "maxgeom" int
  let mjvScene_ngeom = field _mjvScene "ngeom" int
  let mjvScene_geoms = field _mjvScene "geoms" (ptr mjvGeom)
  let mjvScene_geomorder = field _mjvScene "geomorder" (ptr int)
  let mjvScene_nskin = field _mjvScene "nskin" int
  let mjvScene_skinfacenum = field _mjvScene "skinfacenum" (ptr int)
  let mjvScene_skinvertadr = field _mjvScene "skinvertadr" (ptr int)
  let mjvScene_skinvertnum = field _mjvScene "skinvertnum" (ptr int)
  let mjvScene_skinvert = field _mjvScene "skinvert" (ptr float)
  let mjvScene_skinnormal = field _mjvScene "skinnormal" (ptr float)
  let mjvScene_nlight = field _mjvScene "nlight" int
  let mjvScene_lights = field _mjvScene "lights" (ptr mjvLight)
  let mjvScene_camera = field _mjvScene "camera" (ptr mjvGLCamera)
  let mjvScene_enabletransform = field _mjvScene "enabletransform" (ptr uchar)
  let mjvScene_translate = field _mjvScene "translate" (ptr float)
  let mjvScene_rotate = field _mjvScene "rotate" (ptr float)
  let mjvScene_scale = field _mjvScene "scale" float
  let mjvScene_stereo = field _mjvScene "stereo" int
  let mjvScene_flags = field _mjvScene "flags" (ptr uchar)
  let mjvScene_framewidth = field _mjvScene "framewidth" int
  let mjvScene_framergb = field _mjvScene "framergb" (ptr float)
  let () = seal _mjvScene

  type mjvScene = _mjvScene

  let mjvScene = _mjvScene

  type mjvPerturb

  let mjvPerturb : mjvScene structure typ = structure "mjvPerturb"

  type _mjrContext

  let _mjrContext : _mjrContext structure typ = structure "_mjrContext"
  let mjrContext_lineWidth = field _mjrContext "lineWidth" float
  let mjrContext_shadowClip = field _mjrContext "shadowClip" float
  let mjrContext_shadowScale = field _mjrContext "shadowScale" float
  let mjrContext_fogStart = field _mjrContext "fogStart" float
  let mjrContext_fogEnd = field _mjrContext "fogEnd" float
  let mjrContext_fogRGBA = field _mjrContext "fogRGBA" (ptr float)
  let mjrContext_shadowSize = field _mjrContext "shadowSize" int
  let mjrContext_offWidth = field _mjrContext "offWidth" int
  let mjrContext_offHeight = field _mjrContext "offHeight" int
  let mjrContext_offSamples = field _mjrContext "offSamples" int
  let mjrContext_fontScale = field _mjrContext "fontScale" int
  let mjrContext_auxWidth = field _mjrContext "auxWidth" (ptr int)
  let mjrContext_auxHeight = field _mjrContext "auxHeight" (ptr int)
  let mjrContext_auxSamples = field _mjrContext "auxSamples" (ptr int)
  let mjrContext_offFBO = field _mjrContext "offFBO" uint
  let mjrContext_offFBO_r = field _mjrContext "offFBO_r" uint
  let mjrContext_offColor = field _mjrContext "offColor" uint
  let mjrContext_offColor_r = field _mjrContext "offColor_r" uint
  let mjrContext_offDepthStencil = field _mjrContext "offDepthStencil" uint
  let mjrContext_offDepthStencil_r = field _mjrContext "offDepthStencil_r" uint
  let mjrContext_shadowFBO = field _mjrContext "shadowFBO" uint
  let mjrContext_shadowTex = field _mjrContext "shadowTex" uint
  let mjrContext_auxFBO = field _mjrContext "auxFBO" (ptr uint)
  let mjrContext_auxFBO_r = field _mjrContext "auxFBO_r" (ptr uint)
  let mjrContext_auxColor = field _mjrContext "auxColor" (ptr uint)
  let mjrContext_auxColor_r = field _mjrContext "auxColor_r" (ptr uint)
  let mjrContext_ntexture = field _mjrContext "ntexture" int
  let mjrContext_textureType = field _mjrContext "textureType" (ptr int)
  let mjrContext_texture = field _mjrContext "texture" (ptr uint)
  let mjrContext_basePlane = field _mjrContext "basePlane" uint
  let mjrContext_baseMesh = field _mjrContext "baseMesh" uint
  let mjrContext_baseHField = field _mjrContext "baseHField" uint
  let mjrContext_baseBuiltin = field _mjrContext "baseBuiltin" uint
  let mjrContext_baseFontNormal = field _mjrContext "baseFontNormal" uint
  let mjrContext_baseFontShadow = field _mjrContext "baseFontShadow" uint
  let mjrContext_baseFontBig = field _mjrContext "baseFontBig" uint
  let mjrContext_rangePlane = field _mjrContext "rangePlane" int
  let mjrContext_rangeMesh = field _mjrContext "rangeMesh" int
  let mjrContext_rangeHField = field _mjrContext "rangeHField" int
  let mjrContext_rangeBuiltin = field _mjrContext "rangeBuiltin" int
  let mjrContext_rangeFont = field _mjrContext "rangeFont" int
  let mjrContext_nskin = field _mjrContext "nskin" int
  let mjrContext_skinvertVBO = field _mjrContext "skinvertVBO" (ptr uint)
  let mjrContext_skinnormalVBO = field _mjrContext "skinnormalVBO" (ptr uint)
  let mjrContext_skintexcoordVBO = field _mjrContext "skintexcoordVBO" (ptr uint)
  let mjrContext_skinfaceVBO = field _mjrContext "skinfaceVBO" (ptr uint)
  let mjrContext_charWidth = field _mjrContext "charWidth" (ptr int)
  let mjrContext_charWidthBig = field _mjrContext "charWidthBig" (ptr int)
  let mjrContext_charHeight = field _mjrContext "charHeight" int
  let mjrContext_charHeightBig = field _mjrContext "charHeightBig" int
  let mjrContext_glewInitialized = field _mjrContext "glewInitialized" int
  let mjrContext_windowAvailable = field _mjrContext "windowAvailable" int
  let mjrContext_windowSamples = field _mjrContext "windowSamples" int
  let mjrContext_windowStereo = field _mjrContext "windowStereo" int
  let mjrContext_windowDoublebuffer = field _mjrContext "windowDoublebuffer" int
  let mjrContext_currentBuffer = field _mjrContext "currentBuffer" int
  let () = seal _mjrContext

  type mjrContext = _mjrContext

  let mjrContext = _mjrContext

  type _mjrRect

  let _mjrRect : _mjrRect structure typ = structure "_mjrRect"
  let mjrRect_left = field _mjrRect "left" int
  let mjrRect_bottom = field _mjrRect "bottom" int
  let mjrRect_width = field _mjrRect "width" int
  let mjrRect_height = field _mjrRect "height" int
  let () = seal _mjrRect

  type mjrRect = _mjrRect

  let mjrRect = _mjrRect

  type mjtMouse =
    | MOUSE_NONE
    | MOUSE_ROTATE_V
    | MOUSE_ROTATE_H
    | MOUSE_MOVE_V
    | MOUSE_MOVE_H
    | MOUSE_ZOOM
    | MOUSE_SELECT

  let mjMOUSE_NONE = constant "mjMOUSE_NONE" int64_t
  let mjMOUSE_ROTATE_V = constant "mjMOUSE_ROTATE_V" int64_t
  let mjMOUSE_ROTATE_H = constant "mjMOUSE_ROTATE_H" int64_t
  let mjMOUSE_MOVE_V = constant "mjMOUSE_MOVE_V" int64_t
  let mjMOUSE_MOVE_H = constant "mjMOUSE_MOVE_H" int64_t
  let mjMOUSE_ZOOM = constant "mjMOUSE_ZOOM" int64_t
  let mjMOUSE_SELECT = constant "mjMOUSE_SELECT" int64_t

  type mjtCatBit =
    | CAT_STATIC
    | CAT_DYNAMIC
    | CAT_DECOR
    | CAT_ALL

  let mjCAT_STATIC = constant "mjCAT_STATIC" int64_t
  let mjCAT_DYNAMIC = constant "mjCAT_DYNAMIC" int64_t
  let mjCAT_DECOR = constant "mjCAT_DECOR" int64_t
  let mjCAT_ALL = constant "mjCAT_ALL" int64_t

  let mjtCatBit =
    S.enum
      "mjtCatBit"
      ~typedef:true
      [ CAT_STATIC, mjCAT_STATIC
      ; CAT_DYNAMIC, mjCAT_DYNAMIC
      ; CAT_DECOR, mjCAT_DECOR
      ; CAT_ALL, mjCAT_ALL
      ]
      ~unexpected:(fun _ -> failwith "unexpected mjtCAT_BIT element data type enum")


  let mjtMouse =
    S.enum
      "mjtMouse"
      ~typedef:true
      [ MOUSE_NONE, mjMOUSE_NONE
      ; MOUSE_ROTATE_H, mjMOUSE_ROTATE_H
      ; MOUSE_ROTATE_V, mjMOUSE_ROTATE_V
      ; MOUSE_MOVE_H, mjMOUSE_MOVE_H
      ; MOUSE_MOVE_V, mjMOUSE_MOVE_V
      ; MOUSE_ZOOM, mjMOUSE_ZOOM
      ; MOUSE_SELECT, mjMOUSE_SELECT
      ]
      ~unexpected:(fun _ -> failwith "unexpected mjtMOUSE element data type enum")
end
