#!/usr/bin/env python3
import os,sys
import shutil


PSPROJ=os.path.expanduser('~/cic-autosync-oc-binary/if-power-supply/kicad-jail/atmega-powersupply-v1')
MANU='manufacturing'
CURR=os.getcwd()

# https://medium.com/inventhub/better-manage-kicad-projects-using-git-8d06e1310af8
def main ():
    pro_file=os.path.join(PSPROJ,'atmega-powersupply-v1.pro')
    sch_file=os.path.join(PSPROJ,'atmega-powersupply-v1.sch')
    # TODO: cache lib?
    pcb_file=os.path.join(PSPROJ,'atmega-powersupply-v1.kicad_pcb')
    if not os.path.exists(MANU):
        os.mkdir(MANU)
    manufacturing=os.path.join(PSPROJ,MANU)
    for f in [pro_file,sch_file,pcb_file]:
        if not os.path.exists(f):
            print("File doesn't exist: ",f)
            return 0
    for f in [pro_file,sch_file,pcb_file]:
        print("Copying: ",f)
        shutil.copyfile(f,os.path.join(CURR,os.path.split(f)[1]))
    if os.path.exists(manufacturing):
        for f in os.listdir(manufacturing):
            ff=os.path.join(PSPROJ,MANU,f)
            if os.path.isfile(ff) and os.path.splitext(ff)[1] in ['.drl','.gbr','.zip']:
                print("Copying: ",ff)
                shutil.copyfile(ff,os.path.join(CURR,MANU,os.path.split(ff)[1]))
            else:
                print("Not copying: ",ff)
    else:
        print("Path does not exist: ",manufacturing)

if __name__ == '__main__':
    main()
