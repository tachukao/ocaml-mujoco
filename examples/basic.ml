open Wrapper

let model_xml = Cmdargs.(get_string "-xml" |> force ~usage:"model XML")
let model = mj_loadXML model_xml (mjVFS_null ()) "Example" 1000
let data = mj_makeData model

(* Create camera option scene and context *)
let cam = mjvCamera_allocate ()
let opt = mjvOption_allocate ()
let scn = mjvScene_allocate ()
let con = mjrContext_allocate ()
let button_left = ref false
let button_middle = ref false
let button_right = ref false
let lastx = ref 0.
let lasty = ref 0.

(* key callback *)
let key_callback _ k _ act _ =
  if k = GLFW.Backspace && act = GLFW.Press
  then (
    mj_resetData model data;
    mj_forward model data)


(* mouse button callback *)
let mouse_button window _ _ _ =
  (button_left := GLFW.(getMouseButton ~window ~button:GLFW.mouse_button_left));
  (button_middle := GLFW.(getMouseButton ~window ~button:GLFW.mouse_button_middle));
  (button_right := GLFW.(getMouseButton ~window ~button:GLFW.mouse_button_right));
  let x, y = GLFW.getCursorPos ~window in
  lastx := x;
  lasty := y


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
      model
      (mjtMouse_to_int action)
      (dx /. Int.to_float width)
      (dy /. Int.to_float height)
      !&scn
      !&cam)


(* scroll callback *)
let scroll_callback _ _ yoffset =
  mjv_moveCamera model (mjtMouse_to_int MjMOUSE_ZOOM) 0. (-0.05 *. yoffset) !&scn !&cam


let () =
  (* Initialize the GLFW library *)
  GLFW.init ();
  at_exit GLFW.terminate;
  (* Set defaults *)
  mjv_defaultCamera !&cam;
  mjv_defaultOption !&opt;
  mjv_defaultScene !&scn;
  mjr_defaultContext !&con;
  (* Create a windowed mode window and its OpenGL context *)
  let window = GLFW.createWindow ~width:1200 ~height:900 ~title:"Demo" () in
  (* Make the window's context current *)
  GLFW.makeContextCurrent ~window:(Some window);
  (* Make scene and conext *)
  mjv_makeScene model !&scn 2000;
  mjr_makeContext model !&con 150;
  (* Set various callbacks *)
  GLFW.setKeyCallback ~window ~f:(Some key_callback) |> ignore;
  GLFW.setCursorPosCallback ~window ~f:(Some mouse_move) |> ignore;
  GLFW.setMouseButtonCallback ~window ~f:(Some mouse_button) |> ignore;
  GLFW.setScrollCallback ~window ~f:(Some scroll_callback) |> ignore;
  (* Loop until the user closes the window *)
  while not (GLFW.windowShouldClose ~window) do
    let simstart = mjData_get_time Ctypes.(!@data) in
    (* Run the simulation loop *)
    while mjData_get_time Ctypes.(!@data) -. simstart < 1.0 /. 60.0 do
      mj_step model data
    done;
    let mjf_width, mjf_height = GLFW.getFramebufferSize ~window in
    let viewport = mjrRect_make ~mjf_left:0 ~mjf_width ~mjf_bottom:0 ~mjf_height () in
    (* Update Mujoco Scene *)
    mjv_updateScene
      model
      data
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
  done;
  mj_deleteData data;
  mj_deleteModel model
