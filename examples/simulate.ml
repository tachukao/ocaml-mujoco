open Base

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


let () =
  (* print version, check compatibility *)
  Stdio.printf "MuJoCo Pro version %i\n" Mujoco.version;
  Mujoco.delete_data data;
  Mujoco.delete_model model;
  Mujoco.free_vscene scn;
  Mujoco.free_rcontext con
