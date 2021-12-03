(*
open Base

type settings =
  { exit_request : bool ref
  ; vsync : int ref
  }

let settings = { exit_request = ref false; vsync = ref 1 }
let model_xml = Cmdargs.(get_string "-xml" |> force ~usage:"model XML")
let model = Mujoco.load_xml ~name:"Example" model_xml
let data = Mujoco.make_data model
let cam = Mujoco.make_default_vcamera ()
let opt = Mujoco.make_default_voption ()
let scn = Mujoco.make_default_vscene ()
let con = Mujoco.make_default_rcontext ()
let maxgeom = 5000
let syncmisalign = 0.1
let refreshfactor = 0.7

(* help strings *)
let help_content =
  String.concat
    ~sep:""
    [ "Alt mouse button\n"
    ; "UI right hold\n"
    ; "UI title double-click\n"
    ; "Space\n"
    ; "Esc\n"
    ; "Right arrow\n"
    ; "Left arrow\n"
    ; "Down arrow\n"
    ; "Up arrow\n"
    ; "Page Up\n"
    ; "Double-click\n"
    ; "Right double-click\n"
    ; "Ctrl Right double-click\n"
    ; "Scroll, middle drag\n"
    ; "Left drag\n"
    ; "[Shift] right drag\n"
    ; "Ctrl [Shift] drag\n"
    ; "Ctrl [Shift] right drag"
    ]


let help_title =
  String.concat
    ~sep:""
    [ "Swap left-right\n"
    ; "Show UI shortcuts\n"
    ; "Expand/collapse all  \n"
    ; "Pause\n"
    ; "Free camera\n"
    ; "Step forward\n"
    ; "Step back\n"
    ; "Step forward 100\n"
    ; "Step back 100\n"
    ; "Select parent\n"
    ; "Select\n"
    ; "Center\n"
    ; "Track camera\n"
    ; "Zoom\n"
    ; "View rotate\n"
    ; "View translate\n"
    ; "Object rotate\n"
    ; "Object translate"
    ]


let render _ = ()
let drop _ _ = ()
let profilerinit () = ()
let sensorinit () = ()

let init () =
  (* print version, check compatibility *)
  Stdio.printf "MuJoCo Pro version %i\n" Mujoco.version;
  (* Initialize the GLFW library *)
  GLFW.init ();
  Caml.at_exit GLFW.terminate;
  (* multisampling *)
  GLFW.windowHint ~hint:GLFW.Samples ~value:(Some 4);
  GLFW.windowHint ~hint:GLFW.Visible ~value:true;
  (* get videomode and save *)
  let vmode = GLFW.getVideoMode ~monitor:(GLFW.getPrimaryMonitor ()) in
  (* create window *)
  let window =
    GLFW.createWindow
      ~width:(2 * vmode.width / 3)
      ~height:(2 * vmode.height / 3)
      ~title:"Simulate"
      ()
  in
  (* save window position and size *)
  let window_pos = GLFW.getWindowPos ~window in
  let window_size = GLFW.getWindowSize ~window in
  (* make context current, set v-sync *)
  GLFW.makeContextCurrent ~window:(Some window);
  GLFW.swapInterval ~interval:!(settings.vsync);
  (* init abstract visualization *)
  profilerinit ();
  sensorinit ();
  Mujoco.(make_vscene ~model scn maxgeom);
  (* select default font *)
  (*
    int fontscale = uiFontScale(window);
    settings.font = fontscale/50 - 1;
    *)
  (* make empty context *)
  Mujoco.(make_rcontext model con MjFONTSCALE_150);
  (* set GLFW callbacks *)
  (* uiSetCallback(window, &uistate, uiEvent, uiLayout); *)
  GLFW.setWindowRefreshCallback ~window ~f:(Some render) |> ignore;
  GLFW.setDropCallback ~window ~f:(Some drop) |> ignore


(* // init state and uis *)
(* memset(&uistate, 0, sizeof(mjuiState));
    memset(&ui0, 0, sizeof(mjUI));
    memset(&ui1, 0, sizeof(mjUI));
    ui0.spacing = mjui_themeSpacing(settings.spacing);
    ui0.color = mjui_themeColor(settings.color);
    ui0.predicate = uiPredicate;
    ui0.rectid = 1;
    ui0.auxid = 0;
    ui1.spacing = mjui_themeSpacing(settings.spacing);
    ui1.color = mjui_themeColor(settings.color);
    ui1.predicate = uiPredicate;
    ui1.rectid = 2;
    ui1.auxid = 1;
 *)

(*
    (* populate uis with standard sections *)
    mjui_add(&ui0, defFile);
    mjui_add(&ui0, defOption);
    mjui_add(&ui0, defSimulation);
    mjui_add(&ui0, defWatch);
    uiModify(window, &ui0, &uistate, &con);
    uiModify(window, &ui1, &uistate, &con);
    *)

let simulate () = ()
let prepare () = ()

let () =
  init ();
  (* Create a windowed mode window and its OpenGL context *)
  let window = GLFW.createWindow ~width:1200 ~height:900 ~title:"Demo" () in
  (* start simulation thread *)
  let simthread = Thread.create simulate () in
  (* event loop *)
  while not (GLFW.windowShouldClose ~window && not !(settings.exit_request)) do
    (* start exclusive access (block simulation thread) *)
    let mtx = Mutex.create () in
    Mutex.lock mtx;
    (* load model (not on first pass, to show "loading" label) *)
    (* if( settings.loadrequest==1 ) *)
    (* loadmodel(); *)
    (* else if( settings.loadrequest>1 ) *)
    (* settings.loadrequest = 1; *)

    (* Poll for and process events *)
    GLFW.pollEvents ();
    (* prepare to render *)
    prepare ();
    (* end exclusive access (allow simulation thread to run) *)
    Mutex.unlock mtx;
    (* render while simulation is running *)
    render window
  done;
  (* // stop simulation thread *)
  (* settings.exitrequest = 1; *)
  Thread.join simthread;
  Mujoco.delete_data data;
  Mujoco.delete_model model;
  Mujoco.free_vscene scn;
  Mujoco.free_rcontext con

  *)