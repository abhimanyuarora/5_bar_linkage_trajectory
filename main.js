// module aliases
var Engine = Matter.Engine,
    Events = Matter.Events,
    Render = Matter.Render,
    Runner = Matter.Runner,
    Body = Matter.Body,
    Composite = Matter.Composite,
    Composites = Matter.Composites,
    Constraint = Matter.Constraint,
    MouseConstraint = Matter.MouseConstraint,
    Mouse = Matter.Mouse,
    World = Matter.World,
    Bodies = Matter.Bodies,
    Vector = Matter.Vector,
    Common = Matter.Common;

var engine = Engine.create(),
    world = engine.world;

// create renderer
var render = Render.create({
    element: document.body,
    engine: engine,
    options: {
        width: 800,
        height: 600,
        wireframes: false,
        background: '#0f0f13'
    }
});

Render.run(render);

// add bodies
var group = Body.nextGroup(true),

L1_act = 0.09;
L2_act = 0.161;

scale = 1500;

L1 = scale * L1_act,
L2 = scale * L2_act,
width = 20;

var count = 0;

var pendulum = Composite.create();

function linkage(x,y,length,width,inf_inertia) {
    return Bodies.rectangle(x+length/2, y+width/2, length, width, { 
        collisionFilter: { group: group },
        frictionAir: 0,
        chamfer: 1,
        inertia: inf_inertia?Infinity:0,
        // mass: inf_inertia?Infinity:0,
        render: {
            // fillStyle: 'transparent',
            lineWidth: 1,
            strokeStyle: '#ffffff'
        }
    }); 
}

Composite.add(pendulum, linkage(350,100,L1,width,true));
var link0 = pendulum.bodies[0];
Composite.add(pendulum, linkage(350+L1,100,L2,width,false));
var link1 = pendulum.bodies[1];
Body.rotate(link1, 2.1638335924702394, Vector.create(350+L1,100+width/2));
Composite.add(pendulum, linkage(link1.bounds.min.x,link1.bounds.max.y,L2,width,false));
var link2 = pendulum.bodies[2];
Body.rotate(link2, 2*Math.PI-2.1638335924702394, Vector.create(link1.bounds.min.x,link1.bounds.max.y));
Composite.add(pendulum, linkage(350-L1,100,L1,width,true));
var link3 = pendulum.bodies[3];

world.gravity.scale = 0.002;

Composite.addConstraint(pendulum, Constraint.create({ 
    bodyB: link0,
    pointB: { x: -L1*0.5, y: 0 },
    pointA: { x: link0.position.x-L1/2, y: link0.position.y },
    length: 0,
    stiffness: 0.9,
    damping: 1,
    // angularStiffness: 1
    // inertia: Infinity
}));

Composite.addConstraint(pendulum, Constraint.create({ 
    bodyA: link0,
    pointA: { x: -L1*0.5, y: 0 },
    bodyB: link3,
    pointB: { x: L1*0.5, y: 0 },
    length: 0,
    stiffness: 1,
    damping: 1
}));

Composite.addConstraint(pendulum, Constraint.create({ 
    bodyA: link1,
    pointA: { x: -L2*0.5*Math.cos(link1.angle), y: -L2*0.5*Math.sin(link1.angle) },
    bodyB: link0,
    pointB: { x: L1*0.5, y: 0 },
    length: 0,
    stiffness: 0.9,
    damping: 1
}));

Composite.addConstraint(pendulum, Constraint.create({ 
    bodyA: link2,
    pointA: { x: -L2*0.5*Math.cos(link1.angle), y: L2*0.5*Math.sin(link1.angle) },
    bodyB: link1,
    pointB: { x: L2*0.5*Math.cos(link1.angle), y: L2*0.5*Math.sin(link1.angle) },
    length: 0,
    stiffness: 0.9,
    damping: 1,
}));

Composite.addConstraint(pendulum, Constraint.create({ 
    bodyA: link3,
    pointA: { x: -L1*0.5, y: 0 },
    bodyB: link2,
    pointB: { x: L2*0.5*Math.cos(link1.angle), y: -L2*0.5*Math.sin(link1.angle) },
    length: 0,
    stiffness: 0.9,
    damping: 1
}));

World.add(world, pendulum);

var currAngle = [0,0];

function setMotorAngle(n, angle) {
    var dir,link_no,offset;
    if (n == 0) {
        dir = 1;
        offset = 0;
        link_no = 0;
    } else if (n == 1) {
        dir = -1;
        // offset = Math.PI/2;
        link_no = 3;
    } else return;

    Body.rotate(pendulum.bodies[link_no],dir*(angle-currAngle[n]),Vector.create(350,100+width/2));
    currAngle[n] = angle;
}

var write_alpha_beta;

var iter = 0;
var num_pts = alpha_beta.length;

var started = false;

var runner = Runner.create();
var mycanvas,ctx;

params = {
    stance_height: 0.15,
    down_amp: 0.03,
    up_amp: 0.05,
    flight_percent: 0.2,
    step_length: 0.15,
    freq: 0.1
}

var do_trajectory = true;

var interval_len = 10;
var p = 0;

var leg_x_y = [];
var leg_L = [];
var leg_theta = [];
var leg_gamma = [];
var leg_alpha = [];
var leg_beta = [];

// var leg_y = [];
var num_data_pts = 0;
var sin_trajectory_interval;

var test_theta = 30;
var test_gamma = 30+180;
var test_alpha = test_gamma - test_theta - 90;
var test_beta  = test_gamma + test_theta - 90;

// setMotorAngle(0,test_alpha/180*Math.PI);
// setMotorAngle(1,test_beta/180*Math.PI);

setMotorAngle(0,0.159223134);
setMotorAngle(1,-0.710525474);

setTimeout(()=>{
    mycanvas = document.getElementById("mycanvas");
    ctx = mycanvas.getContext("2d");
    if (do_trajectory) {
        sin_trajectory_interval = setInterval(()=>{
            p += params.freq * interval_len / 200;
            var gp = p%1;
            var x,y;
            if (gp < params.flight_percent) {
                x = ((gp/params.flight_percent)*params.step_length) - params.step_length/2.0;
                y = -params.up_amp*Math.sin(Math.PI*gp/params.flight_percent) + params.stance_height;
                // leg_y.push();
            } else {
                var percentBack = (gp-params.flight_percent)/(1.0-params.flight_percent);
                x = -percentBack*params.step_length + params.step_length/2.0;
                y = params.down_amp*Math.sin(Math.PI*percentBack) + params.stance_height;
                // leg_y.push();
            }
            leg_x_y.push([x,y]);

            var L = (x**2 + y**2)**0.5;
            // console.log(L);
            leg_L.push([L]);

            var theta = Math.atan2(x,y);
            leg_theta.push([theta]);

            var cos_param = (L1_act**2 + L**2 - L2_act**2) / (2*L1_act*L);
            // var cos_param = (2*L*L1_act - (4*(L**2)*(L1_act)**2-L1_act**2*(L**2-L2_act**2+L1_act**2))**0.5) / (4 * L1_act**2);
            // console.log(cos_param);
            var gamma;
            if (cos_param < -1.0) {
                gamma = Math.PI;
                // console.log(cos_param, cos_param2);
                console.log("ERROR: L is too small to find valid alpha and beta!");
            } else if (cos_param > 1.0) {
                gamma = 0;
                // console.log(cos_param, cos_param2);
                console.log("ERROR: L is too large to find valid alpha and beta!");
            } else {
                // if (gp < params.flight_percent) {
                    // gamma = Math.PI - Math.acos(cos_param);
                //     console.log(Math.acos(cos_param)/Math.PI*180);
                // } else {
                    gamma = Math.PI - Math.acos(cos_param);
                // }
            }
            leg_gamma.push([gamma]);

            var alpha = gamma - theta - Math.PI/2;
            var beta = gamma + theta - Math.PI/2;
            leg_alpha.push([alpha]);
            leg_beta.push([beta]);

            setMotorAngle(0,alpha);
            setMotorAngle(1,beta);

            ctx.fillStyle = "#ffffff";
            ctx.fillRect(link1.position.x + L2*0.5*Math.cos(link1.angle), link1.position.y + L2*0.5*Math.sin(link1.angle),1,1);

            ctx.fillStyle = "#ff0000";
            ctx.fillRect(350 + x*scale, 100 + y*scale+width/2,1,1);        


            num_data_pts++;
            if (num_data_pts >= Infinity) {
                clearInterval(sin_trajectory_interval);
                console.log("Done");

                var csv_table = [['x','y','L','theta','gamma','alpha','beta']];
                for (var i = 0; i < num_data_pts; i++) {
                    csv_table.push([...leg_x_y[i], ...leg_L[i], ...leg_theta[i], ...leg_gamma[i], ...leg_alpha[i], ...leg_beta[i]]);
                }

                // console.log(csv_table);

                let csvContent = "data:text/csv;charset=utf-8," + csv_table.map(e => e.join(",")).join("\n");
                var encodedUri = encodeURI(csvContent);
                var link = document.createElement("a");
                link.innerText = "Download";
                link.setAttribute("href", encodedUri);
                link.setAttribute("download", "my_data.csv");
                document.body.appendChild(link); // Required for FF

                // link.click(); // This will download the data file named "my_data.csv".

            }
        },interval_len);
    }
},1000);

// create runner
var link_kp = 0.88;
var link_kd = 0.2;
var link_ki = 0.011;
var last_vel = [0,0,0,0];
var cumul_vel = [0,0,0,0];
var last_angle = [0,0,0,0];

function run() {

    // var link0_av = (link0.angle - last_angle[0]);
    // last_angle[0] = link0.angle;
    // // console.log(link0.angularVelocity);
    // Body.setAngularVelocity(link0,-link0_av*link_kp-(link0_av-last_vel[0])*link_kd-cumul_vel[0]*link_ki);
    // last_vel[0] = link0_av;
    // cumul_vel[0] += link0_av;
    // Math.max(cumul_vel[0],1);

    window.requestAnimationFrame(run);
    Engine.update(engine, 1000 / 60);
};
// Runner.run(run, engine);
// Engine.run(engine);
run();