let model_xml = Cmdargs.(get_string "-xml" |> force ~usage:"model XML")
let model = Mujoco.load_xml ~name:"Example" model_xml
let data = Mujoco.make_data model

(* Create camera option scene and context *)
let cam = Mujoco.make_default_vcamera ()
let opt = Mujoco.make_default_voption ()
let scn = Mujoco.make_default_vscene ()
let con = Mujoco.make_default_rcontext ()
let button_left = ref false
let button_middle = ref false
let button_right = ref false
let lastx = ref 0.
let lasty = ref 0.

(* key callback *)
let key_callback _ k _ act _ =
  if k = GLFW.Backspace && act = GLFW.Press
  then (
    Mujoco.reset_data model data;
    Mujoco.forward model data)


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
      let open Mujoco in
      if !button_right
      then if mod_shift then MjMOUSE_MOVE_H else MjMOUSE_MOVE_V
      else if !button_left
      then if mod_shift then MjMOUSE_ROTATE_H else MjMOUSE_ROTATE_V
      else MjMOUSE_ZOOM
    in
    Mujoco.move_camera
      model
      action
      (dx /. Int.to_float width)
      (dy /. Int.to_float height)
      scn
      cam)


(* scroll callback *)
let scroll_callback _ _ yoffset =
  Mujoco.move_camera model Mujoco.MjMOUSE_ZOOM 0. (-0.05 *. yoffset) scn cam


let () =
  (* Initialize the GLFW library *)
  GLFW.init ();
  at_exit GLFW.terminate;
  (* Create a windowed mode window and its OpenGL context *)
  let window = GLFW.createWindow ~width:1200 ~height:900 ~title:"Demo" () in
  (* Make the window's context current *)
  GLFW.makeContextCurrent ~window:(Some window);
  (* Make scene and conext *)
  Mujoco.(make_scene model scn 2000);
  Mujoco.(make_context model con MjFONTSCALE_150);
  (* Set various callbacks *)
  GLFW.setKeyCallback ~window ~f:(Some key_callback) |> ignore;
  GLFW.setCursorPosCallback ~window ~f:(Some mouse_move) |> ignore;
  GLFW.setMouseButtonCallback ~window ~f:(Some mouse_button) |> ignore;
  GLFW.setScrollCallback ~window ~f:(Some scroll_callback) |> ignore;
  (* Loop until the user closes the window *)
  while not (GLFW.windowShouldClose ~window) do
    let simstart = Mujoco.get_data_time data in
    (* Run the simulation loop *)
    while Mujoco.get_data_time data -. simstart < 1.0 /. 60.0 do
      Mujoco.step model data
    done;
    let width, height = GLFW.getFramebufferSize ~window in
    let viewport = Mujoco.make_rrect ~left:0 ~width ~bottom:0 ~height in
    (* Update Mujoco Scene *)
    Mujoco.(update_scene model data opt cam MjCAT_ALL scn);
    (* Render on window  *)
    Mujoco.render viewport scn con;
    (* Swap front and back buffers *)
    GLFW.swapBuffers ~window;
    (* Poll for and process events *)
    GLFW.pollEvents ()
  done;
  Mujoco.delete_data data;
  Mujoco.delete_model model;
  Mujoco.free_vscene scn;
  Mujoco.free_vcamera cam
