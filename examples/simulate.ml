open Wrapper
open Base

let memset =
  let open Ctypes in
  let open Foreign in
  foreign "memset" (ptr void @-> int @-> size_t @-> returning void)


let apple = true

(* UI settings not contained in MuJoCo structures *)

type settings =
  { exitrequest : int Ctypes.ptr
  ; spacing : int Ctypes.ptr
  ; color : int Ctypes.ptr
  ; font : int Ctypes.ptr
  ; ui0 : int Ctypes.ptr
  ; ui1 : int Ctypes.ptr
  ; help : int Ctypes.ptr
  ; info : int Ctypes.ptr
  ; profiler : int Ctypes.ptr
  ; sensor : int Ctypes.ptr
  ; fullscreen : int Ctypes.ptr
  ; vsync : int Ctypes.ptr
  ; busywait : int Ctypes.ptr (* simulation *)
  ; run : int Ctypes.ptr
  ; key : int Ctypes.ptr
  ; loadrequest : int Ctypes.ptr (* watch *)
  ; field : char Ctypes.ptr
  ; index : int Ctypes.ptr (* physics: need sync *)
  ; disable : int Ctypes.ptr
  ; enable : int Ctypes.ptr (* rendering: need sync *)
  ; camera : int Ctypes.ptr
  }

let settings =
  let alloc = Ctypes.allocate Ctypes.int in
  { (* file *)
    exitrequest = alloc 0
  ; (* option *)
    spacing = alloc 0
  ; color = alloc 0
  ; font = alloc 0
  ; ui0 = alloc 1
  ; ui1 = alloc 1
  ; help = alloc 0
  ; info = alloc 0
  ; profiler = alloc 0
  ; sensor = alloc 0
  ; fullscreen = alloc 0
  ; vsync = alloc 1
  ; busywait = alloc 0
  ; (* simulation *)
    run = alloc 1
  ; key = alloc 0
  ; loadrequest = alloc 0
  ; (* watch *)
    field =
      (let arr = Ctypes.CArray.make Ctypes.char 300 in
       let s = "qpos" in
       for i = 0 to String.(length s) - 1 do
         Ctypes.CArray.set arr i s.[i]
       done;
       Ctypes.CArray.start arr)
  ; index = alloc 0
  ; (* physics: need sync *)
    disable = Ctypes.allocate_n Ctypes.int ~count:(mjtDisableBit_to_int MjNDISABLE)
  ; enable = Ctypes.allocate_n Ctypes.int ~count:(mjtEnableBit_to_int MjNENABLE)
  ; (* rendering: need sync *)
    camera = alloc 0
  }


let model_xml = Cmdargs.(get_string "-xml" |> force ~usage:"model XML")
let model = mj_loadXML model_xml (mjVFS_null ()) "Example" 1000
let data = mj_makeData model
let cam = mjvCamera_allocate ()
let opt = mjvOption_allocate ()
let scn = mjvScene_allocate ()
let con = mjrContext_allocate ()
let uistate = mjuiState_allocate ()
let ui0 = mjUI_allocate ()
let ui1 = mjUI_allocate ()
let maxgeom = 5000
let syncmisalign = 0.1
let refreshfactor = 0.7
let window_pos = ref (0, 0)
let window_size = ref (0, 0)

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


let d x1 x2 x3 x4 =
  mjuiDef_make
    ~mjf_type:(mjtItem_to_int x1)
    ~mjf_name:x2
    ~mjf_state:x3
    ~mjf_pdata:Ctypes.null
    ~mjf_other:x4
    ()


let d2 x1 x2 x3 x4 x5 =
  mjuiDef_make
    ~mjf_type:(mjtItem_to_int x1)
    ~mjf_name:x2
    ~mjf_state:x3
    ~mjf_pdata:(Ctypes.to_voidp x4)
    ~mjf_other:x5
    ()


(* file section of UI *)
let defFile =
  let arr = Ctypes.CArray.make Typs.mjuiDef 7 in
  let l =
    [| d MjITEM_SECTION "File" 1 "AF"
     ; d MjITEM_BUTTON "Save xml" 2 ""
     ; d MjITEM_BUTTON "Save mjb" 2 ""
     ; d MjITEM_BUTTON "Print model" 2 "CM"
     ; d MjITEM_BUTTON "Print data" 2 "CD"
     ; d MjITEM_BUTTON "Quit" 1 "CQ"
     ; mjuiDef_make ~mjf_type:(mjtItem_to_int MjITEM_END) ()
    |]
  in
  for i = 0 to 6 do
    Ctypes.CArray.set arr i l.(i)
  done;
  Ctypes.CArray.start arr


(* option section of UI *)
let defOption =
  [ d MjITEM_SECTION "Option" 1 "AO"
  ; d2 MjITEM_SECTION "Color" 1 settings.spacing "Tight\nWide"
  ; d2 MjITEM_SECTION "Color" 1 settings.color "Default\nOrange\nWhite\nBlack"
  ; d2 MjITEM_SECTION "Font" 1 settings.font "50 %\n100 %\n200 %\n250 %\n300 %"
  ; d2 MjITEM_CHECKINT "Left UI (Tab)" 1 settings.ui0 " #258"
  ; d2 MjITEM_CHECKINT "Right UI" 1 settings.ui1 "S#258"
  ; d2 MjITEM_CHECKINT "Help" 2 settings.help " #290"
  ; d2 MjITEM_CHECKINT "Info" 2 settings.info " #291"
  ; d2 MjITEM_CHECKINT "Profiler" 2 settings.profiler " #292"
  ; d2 MjITEM_CHECKINT "Sensor" 2 settings.sensor " #293"
  ; (if apple
    then d2 MjITEM_CHECKINT "Fullscreen" 0 settings.fullscreen " #294"
    else d2 MjITEM_CHECKINT "Fullscreen" 1 settings.fullscreen " #294")
  ; d2 MjITEM_CHECKINT "Vertical Sync" 1 settings.vsync " #295"
  ; d2 MjITEM_CHECKINT "Busy Wait" 1 settings.busywait " #296"
  ; mjuiDef_make ~mjf_type:(mjtItem_to_int MjITEM_END) ()
  ]
  |> mjuiDef_to_carray
  |> Ctypes.CArray.start


(* simulation section of UI *)
let defSimulation =
  [ d MjITEM_SECTION "Simulation" 1 "AS"
  ; d2 MjITEM_RADIO "" 2 settings.run "Pause\nRun"
  ; d MjITEM_BUTTON "Reset" 2 " #259"
  ; d MjITEM_BUTTON "Reload" 2 "CL"
  ; d MjITEM_BUTTON "Align" 2 "CA"
  ; d MjITEM_BUTTON "Copy pose" 2 "CC"
  ; d2 MjITEM_SLIDERINT "Key" 3 settings.key "0 0"
  ; mjuiDef_make
      ~mjf_type:(mjtItem_to_int MjITEM_BUTTON)
      ~mjf_name:"Reset to key"
      ~mjf_state:3
      ()
  ; mjuiDef_make
      ~mjf_type:(mjtItem_to_int MjITEM_BUTTON)
      ~mjf_name:"Set key"
      ~mjf_state:3
      ()
  ; mjuiDef_make ~mjf_type:(mjtItem_to_int MjITEM_END) ()
  ]
  |> mjuiDef_to_carray
  |> Ctypes.CArray.start


(* watch section of UI *)
let defWatch =
  [ d MjITEM_SECTION "Watch" 0 "AW"
  ; d2 MjITEM_EDITTXT "Field" 2 settings.field "qpos"
  ; d2 MjITEM_EDITINT "Index" 2 settings.index "1"
  ; d MjITEM_STATIC "Value" 2 " "
  ; mjuiDef_make ~mjf_type:(mjtItem_to_int MjITEM_END) ()
  ]
  |> mjuiDef_to_carray
  |> Ctypes.CArray.start


let render _ = ()
let drop _ _ = ()
let profilerinit () = ()
let sensorinit () = ()

(* determine enable/disable item state given category *)
let uiPredicate =
  let f category _ =
    match category with
    (* require model *)
    | 2 -> if Ctypes.is_null model then 0 else 1
    (* require model and nkey *)
    | 3 ->
      if (not Ctypes.(is_null model)) && Int.(mjModel_get_nkey !@model > 0) then 1 else 0
    (* require model and paused *)
    | 4 -> if (not Ctypes.(is_null model)) && Int.(!@(settings.run) > 0) then 1 else 0
    | _ -> 1
  in
  let g = Ctypes.(int @-> ptr void @-> returning int) in
  Ctypes.coerce Foreign.(funptr g) Typs.mjfItemEnable f


let init () =
  (* print version, check compatibility *)
  Stdio.printf "MuJoCo Pro version %i\n" 210;
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
  window_pos := GLFW.getWindowPos ~window;
  window_size := GLFW.getWindowSize ~window;
  (* make context current, set v-sync *)
  GLFW.makeContextCurrent ~window:(Some window);
  GLFW.swapInterval ~interval:!@(settings.vsync);
  (* init abstract visualization *)
  mjv_defaultCamera !&cam;
  mjv_defaultOption !&opt;
  profilerinit ();
  sensorinit ();
  mjv_defaultScene !&scn;
  mjv_makeScene model !&scn maxgeom;
  (* select default font *)
  (*
    int fontscale = uiFontScale(window);
    settings.font = fontscale/50 - 1;
    *)
  (* make empty context *)
  mjr_makeContext model !&con (mjtFontScale_to_int MjFONTSCALE_150);
  (* set GLFW callbacks *)
  (* TODO: uiSetCallback(window, &uistate, uiEvent, uiLayout); *)
  let w = Ctypes.to_voidp window in
  Bindings.uiSetCallback (Ctypes.to_voidp window) !@uistate |> ignore;
  GLFW.setWindowRefreshCallback ~window ~f:(Some render) |> ignore;
  GLFW.setDropCallback ~window ~f:(Some drop) |> ignore;
  (* init state and uis *)
  let reset z =
    memset
      Ctypes.(to_voidp (addr z))
      0
      Unsigned.Size_t.(of_int Ctypes.(sizeof (reference_type (addr z))))
  in
  reset uistate;
  reset ui0;
  reset ui1;
  mjUI_set_spacing ui0 (mjui_themeSpacing !@(settings.spacing));
  mjUI_set_color ui0 (mjui_themeColor !@(settings.color));
  mjUI_set_predicate ui0 uiPredicate;
  mjUI_set_rectid ui0 1;
  mjUI_set_auxid ui0 0;
  mjUI_set_spacing ui1 (mjui_themeSpacing !@(settings.spacing));
  mjUI_set_color ui1 (mjui_themeColor !@(settings.color));
  mjUI_set_predicate ui1 uiPredicate;
  mjUI_set_rectid ui1 2;
  mjUI_set_auxid ui1 1;
  (* populate uis with standard sections *)
  mjui_add !&ui0 defFile;
  (* mjui_add !&ui0 defOption; *)
  (* mjui_add !&ui0 defSimulation; *)
  (* mjui_add !&ui0 defWatch; *)
  (* TODO: uiModify(window, &ui0, &uistate, &con); *)
  (* TODO: uiModify(window, &ui1, &uistate, &con); *)
  window


let simulate () = ()
let prepare () = ()

let () =
  let _window = init () in
  (*
  (* start simulation thread *)
  let simthread = Thread.create simulate () in
  (* event loop *)
  while not (GLFW.windowShouldClose ~window && not Int.(!@(settings.exitrequest) = 1)) do
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
    Mutex.unlock mtx
    (* render while simulation is running *)
    (* render window *)
  done;
  (* // stop simulation thread *)
  (* settings.exitrequest = 1; *)
  Thread.join simthread;
  *)
  mj_deleteData data;
  mj_deleteModel model;
  mjv_freeScene !&scn;
  mjr_freeContext !&con
