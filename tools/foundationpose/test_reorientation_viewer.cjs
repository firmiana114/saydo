/* Run with node; no browser or hardware required. */
const fs=require('fs'),vm=require('vm'),assert=require('assert');
const source=fs.readFileSync(__dirname+'/reorientation_viewer.js','utf8');
const M=vm.runInNewContext(source+'\nReorientationMath');
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

const pivot=[.02,-.03,.04];for(const r of [M.identity(),M.euler(30,90,135)]){const o=M.objectOrigin(r,pivot),v=M.apply(r,pivot);assert(v.every((x,k)=>Math.abs(x+o[k])<1e-12),'held TCP must remain at the fixed preview center');}
console.log('Fixed-pivot preview checks passed.');

// A tilted object's selected local axis must remain fixed in world space.
const tilted=M.euler(32,47,81),near=(a,b)=>a.every((v,i)=>Math.abs(v-b[i])<1e-10);
for(const mode of ['x','y','z']){
  const axis=[0,0,0];axis['xyz'.indexOf(mode)]=1;
  const delta=M.drag([100,100],[170,100],800,400,basis,mode);
  const rotated=M.rotate(tilted,delta,mode);rigid(rotated);
  assert(near(M.apply(rotated,axis),M.apply(tilted,axis)),'local '+mode+' axis must keep its world direction');
  assert(!near(M.apply(M.multiply(delta,tilted),axis),M.apply(tilted,axis)),'test must distinguish local from base axes');
}
const keyboard=M.rotate(M.euler(0,0,90),M.axisAngle([1,0,0],5*Math.PI/180),'x');
assert(near(M.angles(keyboard),[5,0,90]),'local X keyboard step after Z=90 degrees');
assert(M.rotate(tilted,right,'free').every((row,i)=>near(row,M.multiply(right,tilted)[i])),'free drag remains screen-relative');
console.log('Local-axis drag/keyboard and unchanged free-rotation checks passed.');
