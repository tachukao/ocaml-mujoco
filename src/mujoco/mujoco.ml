open Base
open Bigarray
open Mujoco_core.Wrapper

type tensor = (float, float64_elt, c_layout) Genarray.t
type camera = mjvCamera
type option = mjvOption
type scene = mjvScene
type context = mjrContext

let string_to_char_ptr s = Ctypes.CArray.of_string s |> Ctypes.CArray.start

exception MjModelParseError

module Model = struct
  type t = { ptr : mjModel ptr }

  let load_xml xml =
    let xml = string_to_char_ptr xml in
    let error_buffer = Ctypes.allocate_n ~count:1000 Ctypes.char in
    let m = mj_loadXML xml (mjVFS_null ()) error_buffer 1000 in
    if Ctypes.is_null m
    then (
      Stdio.printf "MuJoCo Internal error:\n\n%!";
      Ctypes.string_from_ptr error_buffer ~length:1000
      |> String.filter ~f:(fun x -> not Char.(x = '\000'))
      |> Stdio.printf "%s\n%!";
      raise MjModelParseError);
    Caml.Gc.finalise mj_deleteModel m;
    { ptr = m }


  let nv m = mjModel_get_nv !@(m.ptr)
  let nu m = mjModel_get_nu !@(m.ptr)
end

module Data = struct
  type t =
    { ptr : mjData ptr
    ; qpos : tensor
    ; qvel : tensor
    ; ctrl : tensor
    }

  let make m =
    let ptr = mj_makeData Model.(m.ptr) in
    Caml.Gc.finalise mj_deleteData ptr;
    let nq = mjModel_get_nq !@(m.ptr) in
    let qpos_ptr = mjData_get_qpos !@ptr in
    let qpos = Ctypes.(bigarray_of_ptr genarray) [| nq |] float64 qpos_ptr in
    let qvel_ptr = mjData_get_qvel !@ptr in
    let qvel = Ctypes.(bigarray_of_ptr genarray) [| nq |] float64 qvel_ptr in
    let nu = Model.nu m in
    let ctrl_ptr = mjData_get_ctrl !@ptr in
    let ctrl = Ctypes.(bigarray_of_ptr genarray) [| nu |] float64 ctrl_ptr in
    { ptr; qpos; qvel; ctrl }


  let reset m d = mj_resetData Model.(m.ptr) d.ptr
  let time d = mjData_get_time !@(d.ptr)
end

let step m d = mj_step Model.(m.ptr) Data.(d.ptr)
let step1 m d = mj_step1 Model.(m.ptr) Data.(d.ptr)
let step2 m d = mj_step2 Model.(m.ptr) Data.(d.ptr)
let forward m d = mj_forward Model.(m.ptr) Data.(d.ptr)

module Viewport = struct
  type t = mjrRect

  let make ~left ~width ~bottom ~height =
    mjrRect_make ~f_left:left ~f_width:width ~f_bottom:bottom ~f_height:height ()
end

let default_camera () =
  let cam = mjvCamera_allocate () in
  mjv_defaultCamera !&cam;
  cam


let default_option () =
  let opt = mjvOption_allocate () in
  mjv_defaultOption !&opt;
  opt


let default_scene () =
  let scn = mjvScene_allocate () in
  Caml.Gc.finalise mjv_freeScene !&scn;
  mjv_defaultScene !&scn;
  scn


let default_context () =
  let con = mjrContext_allocate () in
  Caml.Gc.finalise mjr_freeContext !&con;
  mjr_defaultContext !&con;
  con


let default_callbacks ~cam ~scn ~model ~data =
  let button_left = ref false in
  let button_middle = ref false in
  let button_right = ref false in
  let lastx = ref 0. in
  let lasty = ref 0. in
  (* key callback *)
  let key_callback _ k _ act _ =
    if Caml.(k = GLFW.Backspace && act = GLFW.Press)
    then (
      Data.reset model data;
      forward model data)
  in
  (* mouse button callback *)
  let mouse_button window _ _ _ =
    (button_left := GLFW.(getMouseButton ~window ~button:GLFW.mouse_button_left));
    (button_middle := GLFW.(getMouseButton ~window ~button:GLFW.mouse_button_middle));
    (button_right := GLFW.(getMouseButton ~window ~button:GLFW.mouse_button_right));
    let x, y = GLFW.getCursorPos ~window in
    lastx := x;
    lasty := y
  in
  (* mouse move callback *)
  let mouse_move window xpos ypos =
    if not ((not !button_left) && (not !button_middle) && not !button_right)
    then (
      let dx = xpos -. !lastx in
      let dy = ypos -. !lasty in
      lastx := xpos;
      lasty := ypos;
      let width, height = GLFW.getWindowSize ~window in
      let mod_shift =
        GLFW.(getKey ~window ~key:GLFW.LeftShift)
        || GLFW.(getKey ~window ~key:GLFW.RightShift)
      in
      let action =
        if !button_right
        then if mod_shift then MjMOUSE_MOVE_H else MjMOUSE_MOVE_V
        else if !button_left
        then if mod_shift then MjMOUSE_ROTATE_H else MjMOUSE_ROTATE_V
        else MjMOUSE_ZOOM
      in
      mjv_moveCamera
        Model.(model.ptr)
        (mjtMouse_to_int action)
        (dx /. Int.to_float width)
        (dy /. Int.to_float height)
        !&scn
        !&cam)
  in
  (* scroll callback *)
  let scroll_callback _ _ yoffset =
    mjv_moveCamera
      Model.(model.ptr)
      (mjtMouse_to_int MjMOUSE_ZOOM)
      0.
      (-0.05 *. yoffset)
      !&scn
      !&cam
  in
  key_callback, mouse_button, mouse_move, scroll_callback


let visualise ~loop model data =
  let width = 1200 in
  let height = 900 in
  let cam = default_camera () in
  let opt = default_option () in
  let scn = default_scene () in
  let con = default_context () in
  let key_callback, mouse_button, mouse_move, scroll_callback =
    default_callbacks ~cam ~scn ~model ~data
  in
  (* Initialize the GLFW library *)
  GLFW.init ();
  Caml.at_exit GLFW.terminate;
  (* Create a windowed mode window and its OpenGL context *)
  let window = GLFW.createWindow ~width ~height ~title:"Demo" () in
  (* Make the window's context current *)
  GLFW.makeContextCurrent ~window:(Some window);
  (* Make scene and conext *)
  mjv_makeScene Model.(model.ptr) !&scn 2000;
  mjr_makeContext Model.(model.ptr) !&con 150;
  (* Set various callbacks *)
  GLFW.setKeyCallback ~window ~f:(Some key_callback) |> ignore;
  GLFW.setCursorPosCallback ~window ~f:(Some mouse_move) |> ignore;
  GLFW.setMouseButtonCallback ~window ~f:(Some mouse_button) |> ignore;
  GLFW.setScrollCallback ~window ~f:(Some scroll_callback) |> ignore;
  (* Loop until the user closes the window *)
  while not (GLFW.windowShouldClose ~window) do
    let t0 = Data.time data in
    (* Run the simulation loop *)
    loop ~t0 model data ();
    let width, height = GLFW.getFramebufferSize ~window in
    let viewport = Viewport.make ~left:0 ~width ~bottom:0 ~height in
    (* Update Mujoco Scene *)
    mjv_updateScene
      model.ptr
      data.ptr
      !&opt
      (mjvPerturb_null ())
      !&cam
      (mjtCatBit_to_int MjCAT_ALL)
      !&scn;
    (* Render on window  *)
    mjr_render viewport !&scn !&con;
    (* Swap front and back buffers *)
    GLFW.swapBuffers ~window;
    (* Poll for and process events *)
    GLFW.pollEvents ()
  done


let record
    ?(width = 1200)
    ?(height = 900)
    ?(camera_scale = 1.5)
    ~duration
    ~fps
    ~advance
    model
    data
    file
  =
  let cam = default_camera () in
  let opt = default_option () in
  let scn = default_scene () in
  let con = default_context () in
  (* Initialize the GLFW library *)
  GLFW.init ();
  Caml.at_exit GLFW.terminate;
  (* Create a windowed mode window and its OpenGL context *)
  GLFW.windowHint ~hint:GLFW.Visible ~value:false;
  GLFW.windowHint ~hint:GLFW.DoubleBuffer ~value:false;
  let window = GLFW.createWindow ~width ~height ~title:"Invisible window" () in
  (* Make the window's context current *)
  GLFW.makeContextCurrent ~window:(Some window);
  (* Make scene and conext *)
  mjv_makeScene Model.(model.ptr) !&scn 2000;
  mjr_makeContext Model.(model.ptr) !&con 150;
  (* center and scale view *)
  let lookat = mjvCamera_get_lookat cam |> Ctypes.(bigarray_of_ptr array1) 3 float64 in
  let stat = !@Ctypes.(model.ptr |-> Typs.mjModel_stat) in
  let center =
    !@Ctypes.(model.ptr |-> Typs.mjModel_stat |-> Typs.mjStatistic_center)
    |> Ctypes.(bigarray_of_ptr array1) 3 float64
  in
  Bigarray.Array1.(blit center lookat);
  mjvCamera_set_distance cam (camera_scale *. mjStatistic_get_extent stat);
  let viewport = mjr_maxViewport !&con in
  let width = mjrRect_get_width viewport in
  let height = mjrRect_get_height viewport in
  let rgb_arr = Ctypes.(CArray.make int8_t (3 * width * height)) in
  let rgb = Ctypes.CArray.start rgb_arr in
  let depth = Ctypes.allocate_n Ctypes.float ~count:(width * height) in
  mjr_setBuffer (mjtFramebuffer_to_int MjFB_OFFSCREEN) !&con;
  let rec aux ch frametime framecount =
    (* Loop until the user closes the window *)
    if Float.(Data.time data < duration)
    then
      if Float.(
           (Data.time data -. frametime > 1. /. Float.(of_int fps)) || frametime = 0.)
      then (
        (* Update Mujoco Scene *)
        mjv_updateScene
          model.ptr
          data.ptr
          !&opt
          (mjvPerturb_null ())
          !&cam
          (mjtCatBit_to_int MjCAT_ALL)
          !&scn;
        (* Render on window  *)
        mjr_render viewport !&scn !&con;
        (* overlay time steps *)
        let overlay = Printf.sprintf "Time = %.3f" Data.(time data) in
        mjr_overlay
          (mjtFont_to_int MjFONT_NORMAL)
          (mjtGridPos_to_int MjGRID_TOPLEFT)
          viewport
          (string_to_char_ptr overlay)
          Ctypes.(from_voidp char null)
          !&con;
        (* read rgb and depth buffers *)
        mjr_readPixels Ctypes.(coerce (ptr int8_t) (ptr uchar) rgb) depth viewport !&con;
        Ctypes.CArray.iter (fun x -> Stdio.Out_channel.output_byte ch x) rgb_arr;
        (* save rgb image *)
        let frametime = Data.time data in
        let framecount = Int.succ framecount in
        if mjr_getError () = 1 then Stdio.printf "x%!" else Stdio.printf ".%!";
        advance model data ();
        aux ch frametime framecount)
      else (
        (* Advance simulation *)
        advance model data ();
        aux ch frametime framecount)
    else Stdio.print_endline "\nFINISHED"
  in
  let tmpdir = Caml.Filename.get_temp_dir_name () in
  let tmpfile = Printf.sprintf "%s/%s" tmpdir "tmp_mujoco_recording.out" in
  Stdio.print_endline "BEGIN RECORDING";
  Stdio.Out_channel.with_file tmpfile ~f:(fun ch -> aux ch 0. 0);
  let command =
    Printf.(
      sprintf
        "ffmpeg -f rawvideo -pixel_format rgb24 -video_size %ix%i -framerate %i -i  %s \
         -vf \"vflip\" %s")
      width
      height
      fps
      tmpfile
      file
  in
  Caml.Sys.command command |> ignore
