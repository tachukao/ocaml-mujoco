open Mujoco
open Wrapper

let model_xml = Cmdargs.(get_string "-xml" |> force ~usage:"model XML")

let make_rect ~left ~width ~bottom ~height =
  let rect = Ctypes.make Typs.mjrRect in
  Ctypes.setf rect Typs.mjrRect_left left;
  Ctypes.setf rect Typs.mjrRect_width width;
  Ctypes.setf rect Typs.mjrRect_bottom bottom;
  Ctypes.setf rect Typs.mjrRect_height height;
  rect


let model =
  Bindings.mj_loadXML model_xml Ctypes.(from_voidp Typs.mjVFS null) "Example" 1000


let data = Bindings.mj_makeData model

(* Create camera option scene and context *)
let cam = Ctypes.make Typs.mjvCamera
let opt = Ctypes.make Typs.mjvOption
let scn = Ctypes.make Typs.mjvScene
let con = Ctypes.make Typs.mjrContext
let button_left = ref false
let button_middle = ref false
let button_right = ref false
let lastx = ref 0.
let lasty = ref 0.

(* key callback *)
let key_callback _ k _ act _ =
  if k = GLFW.Backspace && act = GLFW.Press
  then (
    Bindings.mj_resetData model data;
    Bindings.mj_forward model data)


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
      then if mod_shift then Typs.MOUSE_MOVE_H else Typs.MOUSE_MOVE_V
      else if !button_left
      then if mod_shift then Typs.MOUSE_ROTATE_H else Typs.MOUSE_ROTATE_V
      else Typs.MOUSE_ZOOM
    in
    Bindings.mjv_moveCamera
      model
      action
      (dx /. Int.to_float width)
      (dy /. Int.to_float height)
      Ctypes.(addr scn)
      Ctypes.(addr cam))


(* scroll callback *)
let scroll_callback _ _ yoffset =
  Bindings.mjv_moveCamera
    model
    Typs.MOUSE_ZOOM
    0.
    (-0.05 *. yoffset)
    Ctypes.(addr scn)
    Ctypes.(addr cam)


let () =
  (* Initialize the GLFW library *)
  GLFW.init ();
  at_exit GLFW.terminate;
  (* Set defaults *)
  Bindings.mjv_defaultCamera (Ctypes.addr cam);
  Bindings.mjv_defaultOption (Ctypes.addr opt);
  Bindings.mjv_defaultScene (Ctypes.addr scn);
  Bindings.mjr_defaultContext (Ctypes.addr con);
  (* Create a windowed mode window and its OpenGL context *)
  let window = GLFW.createWindow ~width:1200 ~height:900 ~title:"Demo" () in
  (* Make the window's context current *)
  GLFW.makeContextCurrent ~window:(Some window);
  (* Make scene and conext *)
  Bindings.mjv_makeScene model Ctypes.(addr scn) 2000;
  Bindings.mjr_makeContext model Ctypes.(addr con) Typs.mjFONTSCALE_150;
  (* Set various callbacks *)
  GLFW.setKeyCallback ~window ~f:(Some key_callback) |> ignore;
  GLFW.setCursorPosCallback ~window ~f:(Some mouse_move) |> ignore;
  GLFW.setMouseButtonCallback ~window ~f:(Some mouse_button) |> ignore;
  GLFW.setScrollCallback ~window ~f:(Some scroll_callback) |> ignore;
  (* Loop until the user closes the window *)
  while not (GLFW.windowShouldClose ~window) do
    let simstart = Ctypes.(getf !@data Typs.mjData_time) in
    (* Run the simulation loop *)
    while Ctypes.(getf !@data Typs.mjData_time) -. simstart < 1.0 /. 60.0 do
      Bindings.mj_step model data
    done;
    let width, height = GLFW.getFramebufferSize ~window in
    let viewport = make_rect ~left:0 ~width ~bottom:0 ~height in
    (* Update Mujoco Scene *)
    Bindings.mjv_updateScene
      model
      data
      Ctypes.(addr opt)
      Ctypes.(from_voidp Typs.mjvPerturb null)
      (Ctypes.addr cam)
      Typs.CAT_ALL
      Ctypes.(addr scn);
    (* Render on window  *)
    Bindings.mjr_render viewport Ctypes.(addr scn) Ctypes.(addr con);
    (* Swap front and back buffers *)
    GLFW.swapBuffers ~window;
    (* Poll for and process events *)
    GLFW.pollEvents ()
  done;
  Bindings.mj_deleteData data;
  Bindings.mj_deleteModel model
