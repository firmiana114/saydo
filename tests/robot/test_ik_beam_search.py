"""Exercise the exact SDK-independent native search with a branching fixture."""

from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class BeamSearchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which('g++') or shutil.which('clang++')
        if not compiler:
            raise unittest.SkipTest('C++ compiler unavailable')
        cls.folder = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.folder.cleanup)
        root = Path(cls.folder.name)
        source = root / 'test.cpp'
        cls.binary = root / 'test'
        source.write_text(r'''
#include "saydo/robot/ik/ik_beam_search.h"
#include <cassert>
#include <limits>
int main(){
 using namespace saydo_ik;Joints start{};int calls=0;
 auto candidates=[&](int i,const Joints& q){
  ++calls;std::vector<Joints> out;
  if(i==0){Joints a{},b{};b[0]=5;out={a,b};}
  else if(q[0]==5){Joints good{};good[0]=6;out.push_back(good);}
  return out;
 };
 auto greedy=search(start,{10,10},candidates,[]{},1);assert(!greedy.complete);
 auto beam=search(start,{10,10},candidates,[]{},2);
 assert(beam.complete && beam.frontier[0].path[0][0]==5 && beam.frontier[0].path[1][0]==6);
 auto blocked=search(start,{1,1},candidates,[]{},2);assert(!blocked.complete);
 bool cancelled=false;try{search(start,{10,10},candidates,[]{throw std::runtime_error("cancel");});}catch(...){cancelled=true;}assert(cancelled);
 auto invalid=[](int,const Joints&){Joints a{};a[0]=std::numeric_limits<float>::quiet_NaN();return std::vector<Joints>{a};};
 assert(!search(start,{10},invalid,[]{}).complete);
}
''')
        subprocess.run(
            [
                compiler,
                '-std=c++17',
                '-O2',
                '-I',
                str(Path(__file__).resolve().parents[2]),
                str(source),
                '-o',
                str(cls.binary),
            ],
            check=True,
            capture_output=True,
        )

    def test_branch_survives_greedy_dead_end_and_constraints_remain(self):
        subprocess.run([str(self.binary)], check=True, capture_output=True)
