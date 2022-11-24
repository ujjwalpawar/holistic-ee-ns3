import os 
import pandas as pd


def dlsinr():
    phy = pd.DataFrame(columns=['time', 'cellId', 'IMSI', 'RNTI', 'rsrp', 'sinr', 'ComponentCarrierId'])

    if(os.path.exists('DlRsrpSinrStats.txt')):
        f=open('DlRsrpSinrStats.txt','r')
        lines=f.readlines()
        f.close()
        for i in lines:
            i=i.split("\t")
            if(i[0]=='% time'):
                continue
            else:
                pd.concat([phy, pd.DataFrame([[i[0], i[1], i[2], i[3], i[4], i[5], i[6]]], columns=['time', 'cellId', 'IMSI', 'RNTI', 'rsrp', 'sinr', 'ComponentCarrierId'])], ignore_index=True)
                
    else:
        return
    return phy


print(dlsinr())