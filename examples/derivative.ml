open Mujoco
open Wrapper

let model_xml = Cmdargs.(get_string "-xml" |> force ~usage:"model XML")
let niter = 30
let nwarmup = 3
let nepoch = 20
let nstep = 500
let eps = 1e-6

(*
// worker function for parallel finite-difference computation of derivatives
void worker(const mjModel* m, const mjData* dmain, mjData* d, int id)
{
    int nv = m->nv;

    // allocate stack space for result at center
    mjMARKSTACK
    mjtNum* center = mj_stackAlloc(d, nv);
    mjtNum* warmstart = mj_stackAlloc(d, nv);

    // prepare static schedule: range of derivative columns to be computed by this thread
    int chunk = (m->nv + nthread-1) / nthread;
    int istart = id * chunk;
    int iend = mjMIN(istart + chunk, m->nv);

    // copy state and control from dmain to thread-specific d
    d->time = dmain->time;
    mju_copy(d->qpos, dmain->qpos, m->nq);
    mju_copy(d->qvel, dmain->qvel, m->nv);
    mju_copy(d->qacc, dmain->qacc, m->nv);
    mju_copy(d->qacc_warmstart, dmain->qacc_warmstart, m->nv);
    mju_copy(d->qfrc_applied, dmain->qfrc_applied, m->nv);
    mju_copy(d->xfrc_applied, dmain->xfrc_applied, 6*m->nbody);
    mju_copy(d->ctrl, dmain->ctrl, m->nu);

    // run full computation at center point (usually faster than copying dmain)
    if( isforward )
    {
        mj_forward(m, d);

        // extra solver iterations to improve warmstart (qacc) at center point
        for( int rep=1; rep<nwarmup; rep++ )
            mj_forwardSkip(m, d, mjSTAGE_VEL, 1);
    }
    else
        mj_inverse(m, d);

    // select output from forward or inverse dynamics
    mjtNum* output = (isforward ? d->qacc : d->qfrc_inverse);

    // save output for center point and warmstart (needed in forward only)
    mju_copy(center, output, nv);
    mju_copy(warmstart, d->qacc_warmstart, nv);

    // select target vector and original vector for force or acceleration derivative
    mjtNum* target = (isforward ? d->qfrc_applied : d->qacc);
    const mjtNum* original = (isforward ? dmain->qfrc_applied : dmain->qacc);

    // finite-difference over force or acceleration: skip = mjSTAGE_VEL
    for( int i=istart; i<iend; i++ )
    {
        // perturb selected target
        target[i] += eps;

        // evaluate dynamics, with center warmstart
        if( isforward )
        {
            mju_copy(d->qacc_warmstart, warmstart, m->nv);
            mj_forwardSkip(m, d, mjSTAGE_VEL, 1);
        }
        else
            mj_inverseSkip(m, d, mjSTAGE_VEL, 1);

        // undo perturbation
        target[i] = original[i];

        // compute column i of derivative 2
        for( int j=0; j<nv; j++ )
            deriv[(3*isforward+2)*nv*nv + i + j*nv] = (output[j] - center[j])/eps;
    }

    // finite-difference over velocity: skip = mjSTAGE_POS
    for( int i=istart; i<iend; i++ )
    {
        // perturb velocity
        d->qvel[i] += eps;

        // evaluate dynamics, with center warmstart
        if( isforward )
        {
            mju_copy(d->qacc_warmstart, warmstart, m->nv);
            mj_forwardSkip(m, d, mjSTAGE_POS, 1);
        }
        else
            mj_inverseSkip(m, d, mjSTAGE_POS, 1);

        // undo perturbation
        d->qvel[i] = dmain->qvel[i];

        // compute column i of derivative 1
        for( int j=0; j<nv; j++ )
            deriv[(3*isforward+1)*nv*nv + i + j*nv] = (output[j] - center[j])/eps;
    }

    // finite-difference over position: skip = mjSTAGE_NONE
    for( int i=istart; i<iend; i++ )
    {
        // get joint id for this dof
        int jid = m->dof_jntid[i];

        // get quaternion address and dof position within quaternion (-1: not in quaternion)
        int quatadr = -1, dofpos = 0;
        if( m->jnt_type[jid]==mjJNT_BALL )
        {
            quatadr = m->jnt_qposadr[jid];
            dofpos = i - m->jnt_dofadr[jid];
        }
        else if( m->jnt_type[jid]==mjJNT_FREE && i>=m->jnt_dofadr[jid]+3 )
        {
            quatadr = m->jnt_qposadr[jid] + 3;
            dofpos = i - m->jnt_dofadr[jid] - 3;
        }

        // apply quaternion or simple perturbation
        if( quatadr>=0 )
        {
            mjtNum angvel[3] = {0,0,0};
            angvel[dofpos] = eps;
            mju_quatIntegrate(d->qpos+quatadr, angvel, 1);
        }
        else
            d->qpos[m->jnt_qposadr[jid] + i - m->jnt_dofadr[jid]] += eps;

        // evaluate dynamics, with center warmstart
        if( isforward )
        {
            mju_copy(d->qacc_warmstart, warmstart, m->nv);
            mj_forwardSkip(m, d, mjSTAGE_NONE, 1);
        }
        else
            mj_inverseSkip(m, d, mjSTAGE_NONE, 1);

        // undo perturbation
        mju_copy(d->qpos, dmain->qpos, m->nq);

        // compute column i of derivative 0
        for( int j=0; j<nv; j++ )
            deriv[(3*isforward+0)*nv*nv + i + j*nv] = (output[j] - center[j])/eps;
    }

    mjFREESTACK
}
*)

let worker ~is_forward model data =
  ignore is_forward;
  let nv = Ctypes.(getf !@model Typs.mjModel_nv) in
  ignore nv;
  if is_forward
  then (
    Bindings.mj_forward model data;
    for _ = 0 to nwarmup - 1 do
      Bindings.mj_forwardSkip
        model
        data
        Int64.(to_int Typs.mjSTAGE_VEL)
        Int64.(to_int Typs.mjSENS_ACCELEROMETER)
    done)
  else Bindings.mj_inverse model data


let () =
  Printf.printf "niter   : %d\n" niter;
  Printf.printf "nwarmup : %d\n" nwarmup;
  Printf.printf "nepoch  : %d\n" nepoch;
  Printf.printf "nstep   : %d\n" nstep;
  Printf.printf "eps     : %g\n\n" eps;
  let model =
    Bindings.mj_loadXML model_xml Ctypes.(from_voidp Typs.mjVFS null) "Example" 1000
  in
  let data = Bindings.mj_makeData model in
  let deriv =
    Unsigned.Size_t.(
      of_int
        (6
        * Ctypes.(sizeof double)
        * Ctypes.(getf !@model Typs.mjModel_nv)
        * Ctypes.(getf !@model Typs.mjModel_nv)))
    |> Bindings.mju_malloc
    |> Ctypes.from_voidp Ctypes.double
  in
  (* save solver options *)
  let save_iterations =
    Ctypes.(getf !@(model |-> Typs.mjModel_opt) Typs.mjOption_iterations)
  in
  let save_tolerance =
    Ctypes.(getf !@(model |-> Typs.mjModel_opt) Typs.mjOption_tolerance)
  in
  let nefc = ref 0 in
  for _ = 0 to nepoch - 1 do
    (* set solver options for main simulation *)
    Ctypes.(setf !@(model |-> Typs.mjModel_opt) Typs.mjOption_iterations save_iterations);
    Ctypes.(setf !@(model |-> Typs.mjModel_opt) Typs.mjOption_tolerance save_tolerance);
    (* advance main simulation for nstep *)
    for _ = 0 to nstep - 1 do
      Bindings.mj_step model data
    done;
    (nefc := Ctypes.(getf !@data Typs.mjData_nefc));
    (* set solver options for finite differences *)
    Ctypes.(setf !@(model |-> Typs.mjModel_opt) Typs.mjOption_iterations niter);
    Ctypes.(setf !@(model |-> Typs.mjModel_opt) Typs.mjOption_tolerance 0.);
    (* test forward and inverse *)
    for _ = 0 to 1 do
      (* run worker *)
      worker ~is_forward:true model data
    done
  done;
  Bindings.mju_free Ctypes.(to_voidp deriv);
  Bindings.mj_deleteData data;
  Bindings.mj_deleteModel model
