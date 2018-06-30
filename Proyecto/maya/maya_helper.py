from maya import cmds
shapes = cmds.ls(type = "aiStandard")
for shape in shapes:
    cmds.addAttr(shape, longName = "Kshi", attributeType = "float")
	cmds.setAttr(shape+".Kshi", 20.0)



def CreateSegment(p1, p2, name):
    print(p1)
    print(p2)
    cmds.curve(d = 3, p = [ p1, p2], k = [0, 0, 1, 1], n = name)
    
def CreateRay(p1, d, length, name):
    p2 = [p1[0], p1[1], p1[2]]
    p2[0] = p2[0] + d[0] * length
    p2[1] = p2[1] + d[1] * length
    p2[2] = p2[2] + d[2] * length
    CreateSegment(p1, p2, name)

CreateRay([0,0,0], [1,1,1], 10)

ray = [0.169824868, 0.193840683, 0.329561025]
wr = [0.00291779637, 0.375922292, 0.183573082]
u = [-0.183573082, 0.000000000, 0.00291779637]
v = [-0.00109686470, 0.0337075889, -0.0690092146]
wi = [0.00696562603, 0.375743389, 0.183640376]
p = [0.472229004, 6.98278999, 12.4032135]
CreateRay(p, ray, -10, "ray")
CreateRay(p, wr, -10, "wr")
CreateRay(p, u, -10, "u")
CreateRay(p, v, -10, "v")
CreateRay(p, wi, -10, "wi")