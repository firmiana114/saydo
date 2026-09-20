/* Run with node; no browser or hardware required. */
const fs=require('fs'),vm=require('vm'),assert=require('assert');
const source=fs.readFileSync(__dirname+'/placement_viewer.js','utf8');
const M=vm.runInNewContext(source+'\nPlacementMath');
const basis=[[Math.SQRT1_2,Math.SQRT1_2,0],[-1/Math.sqrt(6),1/Math.sqrt(6),2/Math.sqrt(6)],[1/Math.sqrt(3),-1/Math.sqrt(3),1/Math.sqrt(3)]];
function rigid(r){for(let i=0;i<3;i++)for(let j=0;j<3;j++)assert(Math.abs(M.dot(r[i],r[j])-(i===j?1:0))<1e-10);assert(Math.abs(M.dot(M.cross(r[0],r[1]),r[2])-1)<1e-10);}
const still=M.drag([400,200],[400,200],800,400,basis);rigid(still);assert(Math.abs(still[0][0]-1)<1e-12);
const right=M.drag([400,200],[460,200],800,400,basis);rigid(right);assert(M.dot(M.apply(right,basis[2]),basis[0])>0,'rightward drag must move the front surface right on screen');
const up=M.drag([400,200],[400,140],800,400,basis);assert(M.dot(M.apply(up,basis[2]),basis[1])>0);
for(const axis of ['x','y','z']){const r=M.drag([0,0],[150,0],800,400,basis,axis);rigid(r);const v=[0,0,0];v['xyz'.indexOf(axis)]=1;assert(M.apply(r,v).every((x,i)=>Math.abs(x-v[i])<1e-12));}
const fine=M.drag([400,200],[460,200],800,400,basis,'free',true);assert(M.dot(M.apply(fine,basis[2]),basis[0])<M.dot(M.apply(right,basis[2]),basis[0]));
let r=M.identity();for(let i=0;i<1500;i++){r=M.multiply(M.drag([i%700,i%300],[(i*7)%700,(i*11)%300],800,400,basis),r);}rigid(r);
for(const angles of [[0,0,0],[23,-51,137],[0,90,40],[0,-90,-30]]){const r=M.euler(...angles),back=M.euler(...M.angles(r));assert(r.every((row,i)=>row.every((x,j)=>Math.abs(x-back[i][j])<1e-8)));}
console.log('7 viewer math checks passed: identity, screen direction, constrained axes, fine drag, rotation stability, Euler round-trip.');
