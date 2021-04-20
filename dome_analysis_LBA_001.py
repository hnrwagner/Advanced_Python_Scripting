#python #abaqus #abaqustutorial #hnrwagner 


from abaqus import *
from abaqusConstants import *
import regionToolset
import __main__
import section
import regionToolset
import part
import material
import assembly
import step
import interaction
import load
import mesh
import job
import sketch
import visualization
import xyPlot
import connectorBehavior
import odbAccess
from operator import add
import numpy as np



# functions



def Create_Part_3D_Cylinder(radius,length,thickness,part,model):
    s1 = mdb.models[model].ConstrainedSketch(name='__profile__', sheetSize=200.0)
    g, v, d, c = s1.geometry, s1.vertices, s1.dimensions, s1.constraints
    s1.setPrimaryObject(option=STANDALONE)
    s1.CircleByCenterPerimeter(center=(0.0, 0.0), point1=(radius, 0.0))
    s1.CircleByCenterPerimeter(center=(0.0, 0.0), point1=(radius-thickness, 0.0))
    p = mdb.models[model].Part(name=part, dimensionality=THREE_D, type=DEFORMABLE_BODY)
    p = mdb.models[model].parts[part]
    p.BaseSolidExtrude(sketch=s1, depth=length)
    s1.unsetPrimaryObject()
    p = mdb.models[model].parts[part]
    del mdb.models[model].sketches['__profile__']
    
#----------------------------------------------------------------------------

def Create_Part_3D_Hemi_Sphere(radius,thickness,part,model):    
    s = mdb.models[model].ConstrainedSketch(name='__profile__', sheetSize=5000.0)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    s.setPrimaryObject(option=STANDALONE)
    s.ConstructionLine(point1=(0.0, -2500.0), point2=(0.0, 2500.0))
    s.FixedConstraint(entity=g[2])
    s.ArcByCenterEnds(center=(0.0, 0.0), point1=(radius, 0.0), point2=(0.0, radius), direction=COUNTERCLOCKWISE)
    s.ArcByCenterEnds(center=(0.0, 0.0), point1=(radius-thickness, 0.0), point2=(0.0, radius-thickness), direction=COUNTERCLOCKWISE)
    s.Line(point1=(radius-thickness, 0.0), point2=(radius, 0.0))
    s.HorizontalConstraint(entity=g[5], addUndoState=False)
    s.Line(point1=(0.0, radius-thickness), point2=(0.0, radius))
    s.VerticalConstraint(entity=g[6], addUndoState=False)
    p = mdb.models[model].Part(name=part, dimensionality=THREE_D, type=DEFORMABLE_BODY)
    p = mdb.models[model].parts[part]
    p.BaseSolidRevolve(sketch=s, angle=360.0, flipRevolveDirection=OFF)
    s.unsetPrimaryObject()
    p = mdb.models[model].parts[part]
    del mdb.models[model].sketches['__profile__']
    
    
#----------------------------------------------------------------------------

def Create_Part_2D_Stiffened_Sphere(radius,angle,height,anz,part,model): 
    s = mdb.models[model].ConstrainedSketch(name='__profile__', sheetSize=5000.0)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    s.setPrimaryObject(option=STANDALONE)
    s.ConstructionLine(point1=(0.0, -2500.0), point2=(0.0, 2500.0))
    s.FixedConstraint(entity=g[2])
    s.ArcByCenterEnds(center=(0.0, 0.0), point1=(radius*np.sin(angle*np.pi/180.0), radius*np.cos(angle*np.pi/180.0)), point2=(0.0, radius), direction=COUNTERCLOCKWISE)
    s.Line(point1=(0.0, radius), point2=(0.0, radius+height))
    s.radialPattern(geomList=(g[4], ), vertexList=(), number=anz, totalAngle=-angle, centerPoint=(0.0, 0.0))
    s.delete(objectList=(g[4], ))
    s.unsetPrimaryObject()
    # s.HorizontalConstraint(entity=g[4], addUndoState=False)
    # # s.PerpendicularConstraint(entity1=g[3], entity2=g[4], addUndoState=False)
    # s.radialPattern(geomList=(g[4], ), vertexList=(), number=anz, totalAngle=angle, centerPoint=(0.0, 0.0))
    p = mdb.models[model].Part(name=part, dimensionality=THREE_D, type=DEFORMABLE_BODY)
    p = mdb.models[model].parts[part]
    p.BaseShellRevolve(sketch=s, angle=360.0, flipRevolveDirection=OFF)
    s.unsetPrimaryObject()
    p = mdb.models[model].parts[part]
    session.viewports['Viewport: 1'].setValues(displayedObject=p)
    del mdb.models[model].sketches['__profile__']

#----------------------------------------------------------------------------

def Create_Part_2D_Sphere(radius,angle,height,part,model): 
    s = mdb.models[model].ConstrainedSketch(name='__profile__', sheetSize=5000.0)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    s.setPrimaryObject(option=STANDALONE)
    s.ConstructionLine(point1=(0.0, -2500.0), point2=(0.0, 2500.0))
    s.FixedConstraint(entity=g[2])
    s.ArcByCenterEnds(center=(0.0, 0.0), point1=(radius*np.sin(angle*np.pi/180.0), radius*np.cos(angle*np.pi/180.0)), point2=(0.0, radius), direction=COUNTERCLOCKWISE)
    # s.Line(point1=(0.0, radius), point2=(0.0, radius+height))
    # s.radialPattern(geomList=(g[4], ), vertexList=(), number=anz, totalAngle=-angle, centerPoint=(0.0, 0.0))
    # s.delete(objectList=(g[4], ))
    # s.unsetPrimaryObject()
    # s.HorizontalConstraint(entity=g[4], addUndoState=False)
    # # s.PerpendicularConstraint(entity1=g[3], entity2=g[4], addUndoState=False)
    # s.radialPattern(geomList=(g[4], ), vertexList=(), number=anz, totalAngle=angle, centerPoint=(0.0, 0.0))
    p = mdb.models[model].Part(name=part, dimensionality=THREE_D, type=DEFORMABLE_BODY)
    p = mdb.models[model].parts[part]
    p.BaseShellRevolve(sketch=s, angle=360.0, flipRevolveDirection=OFF)
    s.unsetPrimaryObject()
    p = mdb.models[model].parts[part]
    session.viewports['Viewport: 1'].setValues(displayedObject=p)
    del mdb.models[model].sketches['__profile__']

def Create_Part_2D_Sphere_Stiffener(radius,angle,height,part,model):
    s = mdb.models[model].ConstrainedSketch(name='__profile__', sheetSize=5000.0)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    s.setPrimaryObject(option=STANDALONE)
    s.ArcByCenterEnds(center=(0.0, 0.0), point1=(radius*np.sin(angle*np.pi/180.0),        radius*np.cos(angle*np.pi/180.0)), point2=(0.0, radius       ), direction=COUNTERCLOCKWISE)
    s.ArcByCenterEnds(center=(0.0, 0.0), point1=(radius*np.sin(angle*np.pi/180.0)+height, radius*np.cos(angle*np.pi/180.0)), point2=(0.0, radius+height), direction=COUNTERCLOCKWISE)
    s.Line(point1=(radius*np.sin(angle*np.pi/180.0), radius*np.cos(angle*np.pi/180.0)), point2=(radius*np.sin(angle*np.pi/180.0)+height, radius*np.cos(angle*np.pi/180.0)))
    s.HorizontalConstraint(entity=g[4], addUndoState=False)
    # s.PerpendicularConstraint(entity1=g[2], entity2=g[4], addUndoState=False)
    s.Line(point1=(0.0, radius), point2=(0.0, radius+height))
    s.VerticalConstraint(entity=g[5], addUndoState=False)
    s.PerpendicularConstraint(entity1=g[2], entity2=g[5], addUndoState=False)
    p = mdb.models[model].Part(name=part, dimensionality=THREE_D, type=DEFORMABLE_BODY)
    p = mdb.models[model].parts[part]
    p.BaseShell(sketch=s)
    s.unsetPrimaryObject()
    p = mdb.models[model].parts[part]
    del mdb.models[model].sketches['__profile__']
    # s = mdb.models['GNA'].ConstrainedSketch(name='__profile__', sheetSize=200.0)
    # g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    # s.setPrimaryObject(option=STANDALONE)
    # s.ArcByCenterEnds(center=(0.0, 0.0), point1=(30.0, 10.0), point2=(0.0, 31.25), direction=COUNTERCLOCKWISE)
    # s.ArcByCenterEnds(center=(0.0, 0.0), point1=(35.0, 10.0), point2=(0.0, 36.25), direction=COUNTERCLOCKWISE)
    # s.Line(point1=(30.0, 10.0), point2=(35.0, 10.0))
    # s.HorizontalConstraint(entity=g[4], addUndoState=False)
    # s.Line(point1=(0.0, 31.6227766016838), point2=(0.0, 36.4005494464026))
    # s.VerticalConstraint(entity=g[5], addUndoState=False)
    # s.PerpendicularConstraint(entity1=g[2], entity2=g[5], addUndoState=False)
    # p = mdb.models['GNA'].Part(name='Part-3', dimensionality=THREE_D, type=DEFORMABLE_BODY)
    # p = mdb.models['GNA'].parts['Part-3']
    # p.BaseShell(sketch=s)
    # s.unsetPrimaryObject()
    # p = mdb.models['GNA'].parts['Part-3']
    # session.viewports['Viewport: 1'].setValues(displayedObject=p)
    # del mdb.models['GNA'].sketches['__profile__']





#----------------------------------------------------------------------------

def Create_Datum_Plane_by_Principal(type_plane,part,model,offset_plane):
    p = mdb.models[model].parts[part]
    myPlane = p.DatumPlaneByPrincipalPlane(principalPlane=type_plane, offset=offset_plane)
    myID = myPlane.id
    return myID


def Create_Set_All_Cells(model,part,set_name):
    p = mdb.models[model].parts[part]
    c = p.cells[:]
    p.Set(cells=c, name=set_name)

#----------------------------------------------------------------------------

def Create_Set_All_Faces(model,part,set_name):
    p = mdb.models[model].parts[part]
    f = p.faces[:]
    p.Set(faces=f, name=set_name)    

#----------------------------------------------------------------------------

def Create_Material_Data(model,material_name,e11,e22,e33,nu12,nu13,nu23,g12,g13,g23,lts,lcs,tts,tcs,lss,tss):
    mdb.models[model].Material(name=material_name)
    mdb.models[model].materials[material_name].Elastic(type=ENGINEERING_CONSTANTS, table=((e11,e22,e33,nu12,nu13,nu23,g12,g13,g23), ))
    mdb.models[model].materials[material_name].HashinDamageInitiation(table=((lts,lcs,tts,tcs,lss,tss), ))

def Create_Set_Face(x,y,z,model,part,set_name):
    face = ()
    p = mdb.models[model].parts[part]
    f = p.faces
    myFace = f.findAt((x,y,z),)
    face = face + (f[myFace.index:myFace.index+1], )
    p.Set(faces=face, name=set_name)
    return myFace

def Create_Set_Edge(x,y,z,model,part,set_name):
    edge = ()
    p = mdb.models[model].parts[part]
    e = p.edges
    myEdge = e.findAt((x,y,z),)
    edge = edge + (e[myEdge.index:myEdge.index+1], )
    f = p.Set(edges=edge, name=set_name)
    return myEdge

#-----------------------------------------------------------------------------

def Create_Set_Edge_Multi(x,y,z,x2,y2,z2,model,part,set_name):
    edge = ()
    p = mdb.models[model].parts[part]
    e = p.edges
    myEdge = e.findAt((x,y,z),)
    edge = edge + (e[myEdge.index:myEdge.index+1], )
    f = p.Set(edges=edge, name=set_name)
    myEdge = e.findAt((x2,y2,z2),)
    edge = edge + (e[myEdge.index:myEdge.index+1], )
    f = p.Set(edges=edge, name=set_name)
    return myEdge

#-----------------------------------------------------------------------------
def Create_Set_Vertice(x,y,z,model,part,set_name):
    vertice = ()
    p = mdb.models[model].parts[part]
    v = p.vertices
    myVertice = v.findAt((x,y,z),)
    vertice = vertice + (v[myVertice.index:myVertice.index+1], )
    p.Set(vertices=vertice, name=set_name)  
       

def Create_Set_Surface(x,y,z,model,part,set_name):
    face = ()
    p = mdb.models[model].parts[part]
    s = p.faces
    myFace = s.findAt((x,y,z),)
    face = face + (s[myFace.index:myFace.index+1], )
    p.Surface(side1Faces=face, name=set_name)
   

def Create_Assembly(model,part,instance_name):
    a = mdb.models[model].rootAssembly
    a.DatumCsysByDefault(CARTESIAN)
    p = mdb.models[model].parts[part]
    a.Instance(name=instance_name, part=p, dependent=ON)

#-------------------------------------------------------------

def Create_Reference_Point(x,y,z,model,setname):
    a = mdb.models[model].rootAssembly
    myRP = a.ReferencePoint(point=(x, y, z))
    r = a.referencePoints
    myRP_Position = r.findAt((x, y, z),)
    refPoints1=(myRP_Position, )
    a.Set(referencePoints=refPoints1, name=setname)
    return myRP,myRP_Position

def Create_Constraint_Equation(model,constraint_name,set_name,set_name_rp):
    mdb.models[model].Equation(name=constraint_name, terms=((1.0, set_name, 2), (-1.0, set_name_rp, 2)))


def Create_Boundary_Condition_by_Instance(model,instance_name,set_name,BC_name,step_name,u1_BC,u2_BC,u3_BC,ur1_BC,ur2_BC,ur3_BC):
    a = mdb.models[model].rootAssembly
    region = a.instances[instance_name].sets[set_name]
    mdb.models[model].DisplacementBC(name=BC_name, createStepName=step_name, region=region, u1=u1_BC, u2=u2_BC, u3=u3_BC, ur1=ur1_BC, ur2=ur2_BC, ur3=ur3_BC, amplitude=UNSET, distributionType=UNIFORM, fieldName='', localCsys=None)  


def Create_Boundary_Condition_by_RP(model,RP_name,BC_name,step_name,u1_BC,u2_BC,u3_BC,ur1_BC,ur2_BC,ur3_BC):
    a = mdb.models[model].rootAssembly
    region = a.sets[RP_name]
    mdb.models[model].DisplacementBC(name=BC_name, createStepName=step_name, region=region, u1=u1_BC, u2=u2_BC, u3=u3_BC, ur1=ur1_BC, ur2=ur2_BC, ur3=ur3_BC, amplitude=UNSET, distributionType=UNIFORM, fieldName='', localCsys=None)  


def Create_Analysis_Step(model,step_name,pre_step_name,Initial_inc,Max_inc,Min_inc,Inc_Number,NL_ON_OFF):
    a = mdb.models[model].StaticStep(name=step_name, previous=pre_step_name, initialInc=Initial_inc, maxInc=Max_inc, minInc=Min_inc)
    a = mdb.models[model].steps[step_name].setValues(maxNumInc=Inc_Number)
    a = mdb.models[model].steps[step_name].setValues(nlgeom=NL_ON_OFF)
    a = mdb.models[model].steps[step_name].setValues(stabilizationMagnitude=1E-009, stabilizationMethod=DAMPING_FACTOR, continueDampingFactors=False, adaptiveDampingRatio=None)
    a = mdb.models[model].fieldOutputRequests['F-Output-1'].setValues(variables=('S', 'PE', 'PEEQ', 'PEMAG', 'LE', 'U', 'RF', 'CF', 'P', 'CSTRESS', 'CDISP', 'HSNFTCRT', 'HSNFCCRT', 'HSNMTCRT', 'HSNMCCRT'))


#-----------------------------------------------------------------------------

def Create_Riks_Analysis_Step(model):
    a = mdb.models[model].StaticRiksStep(name='Step-1', previous='Initial', maxLPF=1.0, maxNumInc=300, initialArcInc=0.001, minArcInc=1e-50, maxArcInc=0.01, nlgeom=ON)
    a = mdb.models[model].fieldOutputRequests['F-Output-1'].setValues(variables=('S', 'PE', 'PEEQ', 'PEMAG', 'LE', 'U', 'RF', 'CF', 'P', 'CSTRESS', 'CDISP', 'HSNFTCRT', 'HSNFCCRT', 'HSNMTCRT', 'HSNMCCRT'))


#-----------------------------------------------------------------------------

def Create_LBA_Step(model):
    mdb.models[model].BuckleStep(name='Step-1', previous='Initial', numEigen=10, vectors=18, maxIterations=3000)

#-----------------------------------------------------------------------------

def Create_Partion_by_Plane(model,part,id_plane):
    p = mdb.models[model].parts[part]
    c = p.cells[:]
    d = p.datums
    p.PartitionCellByDatumPlane(datumPlane=d[id_plane], cells=c)

#-----------------------------------------------------------------------------

def Create_Composite_Layup(model,part,set_name,composite_name,number,material,thickness,angle,surf,edge):
    layupOrientation = None
    p = mdb.models[model].parts[part]
    region1=p.sets[set_name]
    normalAxisRegion = p.surfaces[surf]
    primaryAxisRegion = p.sets[edge]
    compositeLayup = mdb.models[model].parts[part].CompositeLayup(name=composite_name, description='', elementType=CONTINUUM_SHELL, symmetric=False)
    compositeLayup.Section(preIntegrate=OFF, integrationRule=SIMPSON, poissonDefinition=DEFAULT, thicknessModulus=None, temperature=GRADIENT, useDensity=OFF)
    for i in range(0,number,1):
        compositeLayup.CompositePly(suppressed=False, plyName='Ply-'+str(i), region=region1, material=material, thicknessType=SPECIFY_THICKNESS, thickness=thickness, orientationType=SPECIFY_ORIENT, orientationValue=angle[i], additionalRotationType=ROTATION_NONE, additionalRotationField='', axis=AXIS_3, angle=0.0, numIntPoints=3)
        compositeLayup.ReferenceOrientation(orientationType=DISCRETE, localCsys=None, additionalRotationType=ROTATION_ANGLE, angle=90.0, additionalRotationField='', axis=AXIS_3, stackDirection=STACK_3, normalAxisDefinition=SURFACE, normalAxisRegion=normalAxisRegion, normalAxisDirection=AXIS_3, flipNormalDirection=False, primaryAxisDefinition=EDGE, primaryAxisRegion=primaryAxisRegion, primaryAxisDirection=AXIS_2, flipPrimaryDirection=False)
   
#-----------------------------------------------------------------------------

def Create_Sandwich_Composite_Layup(model,part,set_name,composite_name,number,material,material_2,thickness,thickness_2,angle,surf,edge):
    layupOrientation = None
    p = mdb.models[model].parts[part]
    region1=p.sets[set_name]
    normalAxisRegion = p.surfaces[surf]
    primaryAxisRegion = p.sets[edge]
    compositeLayup = mdb.models[model].parts[part].CompositeLayup(name=composite_name, description='', elementType=CONTINUUM_SHELL, symmetric=False)
    compositeLayup.Section(preIntegrate=OFF, integrationRule=SIMPSON, poissonDefinition=DEFAULT, thicknessModulus=None, temperature=GRADIENT, useDensity=OFF)
    for i in range(0,number,1):
        if (angle[i] == 1):
            compositeLayup.CompositePly(suppressed=False, plyName='Ply-'+str(i), region=region1, material=material_2, thicknessType=SPECIFY_THICKNESS, thickness=thickness_2, orientationType=SPECIFY_ORIENT, orientationValue=angle[i], additionalRotationType=ROTATION_NONE, additionalRotationField='', axis=AXIS_3, angle=0.0, numIntPoints=3)
            compositeLayup.ReferenceOrientation(orientationType=DISCRETE, localCsys=None, additionalRotationType=ROTATION_ANGLE, angle=90.0, additionalRotationField='', axis=AXIS_3, stackDirection=STACK_3, normalAxisDefinition=SURFACE, normalAxisRegion=normalAxisRegion, normalAxisDirection=AXIS_3, flipNormalDirection=False, primaryAxisDefinition=EDGE, primaryAxisRegion=primaryAxisRegion, primaryAxisDirection=AXIS_2, flipPrimaryDirection=False)
        else:
            compositeLayup.CompositePly(suppressed=False, plyName='Ply-'+str(i), region=region1, material=material, thicknessType=SPECIFY_THICKNESS, thickness=thickness, orientationType=SPECIFY_ORIENT, orientationValue=angle[i], additionalRotationType=ROTATION_NONE, additionalRotationField='', axis=AXIS_3, angle=0.0, numIntPoints=3)
            compositeLayup.ReferenceOrientation(orientationType=DISCRETE, localCsys=None, additionalRotationType=ROTATION_ANGLE, angle=90.0, additionalRotationField='', axis=AXIS_3, stackDirection=STACK_3, normalAxisDefinition=SURFACE, normalAxisRegion=normalAxisRegion, normalAxisDirection=AXIS_3, flipNormalDirection=False, primaryAxisDefinition=EDGE, primaryAxisRegion=primaryAxisRegion, primaryAxisDirection=AXIS_2, flipPrimaryDirection=False)
   
#----------------------------------------------------------------------------

def Create_Mesh(model,part,size):
    p = mdb.models[model].parts[part]
    elemType1 = mesh.ElemType(elemCode=S4R, elemLibrary=STANDARD, secondOrderAccuracy=OFF, hourglassControl=DEFAULT)
    elemType2 = mesh.ElemType(elemCode=S3, elemLibrary=STANDARD)
    faces = p.faces[:]
    pickedRegions =(faces, )
    p.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2))
    p.seedPart(size=size, deviationFactor=0.1, minSizeFactor=0.1)
    p.generateMesh()


def Create_SPLA(model,instance_name,set_name,load_name,step_name,load):
    a = mdb.models[model].rootAssembly
    region = a.instances[instance_name].sets[set_name]
    mdb.models[model].ConcentratedForce(name=load_name, createStepName=step_name, region=region, cf2=load, distributionType=UNIFORM, field='', localCsys=None)


def Create_Pressure_Load(model,instance_name,load_name,step_name,surface,load):
    a = mdb.models[model].rootAssembly
    region = a.instances[instance_name].surfaces[surface]
    mdb.models[model].Pressure(name=load_name, createStepName=step_name, region=region, distributionType=UNIFORM, field='', magnitude=load, amplitude=UNSET)    

def CreateCutout(model,part,radius_cutout,id_plane,edge,x,y,z):
    p = mdb.models[model].parts[part]
    e, d = p.edges, p.datums
    t = p.MakeSketchTransform(sketchPlane=d[id_plane], sketchUpEdge=edge, sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, origin=(x, y, z))
    s = mdb.models[model].ConstrainedSketch(name='__profile__', sheetSize=2000.0, gridSpacing=20.0, transform=t)
    g, v, d1, c = s.geometry, s.vertices, s.dimensions, s.constraints
    s.setPrimaryObject(option=SUPERIMPOSE)
    p.projectReferencesOntoSketch(sketch=s, filter=COPLANAR_EDGES)
    s.CircleByCenterPerimeter(center=(0.0, 0.0), point1=(radius_cutout, 0.0))
    p.CutExtrude(sketchPlane=d[id_plane], sketchUpEdge=edge, sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, sketch=s, flipExtrudeDirection=ON)
    s.unsetPrimaryObject()
    del mdb.models[model].sketches['__profile__']

def AssignStack(model,part,face):
    p = mdb.models[model].parts[part]
    c = p.cells[:]
    p.assignStackDirection(referenceRegion=face, cells=c)

def CreateJob(model,job_name,cpu):
    a = mdb.models[model].rootAssembly
    mdb.Job(name=job_name, model=model, description='', type=ANALYSIS, atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF, modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', scratch='', resultsFormat=ODB, multiprocessingMode=DEFAULT, numCpus=cpu, numDomains=cpu, numGPUs=0)

def SubmitJob(job_name):
    mdb.jobs[job_name].submit(consistencyChecking=OFF)

def Linear_Pattern_Instance(model,instance,length,anz):
    a = mdb.models[model].rootAssembly
    a.LinearInstancePattern(instanceList=(instance, ), direction1=(1.0, 0.0, 0.0), direction2=(0.0, 0.0, 1.0), number1=1, number2=anz, spacing1=2740.0, spacing2=length)

#----------------------------------------------------------------------------
def Radial_Pattern_Instance(model,instance,x,y,z,a_x,a_y,a_z,anz,angle):
    a = mdb.models[model].rootAssembly
    a.RadialInstancePattern(instanceList=(instance, ), point=(x,y,z), axis=(a_x,a_y,a_z), number=anz, totalAngle=angle)

#----------------------------------------------------------------------------

def Rotate_Instance(model,instance,x,y,z,a_x,a_y,a_z,angle):
    a = mdb.models[model].rootAssembly
    a.rotate(instanceList=(instance, ), axisPoint=(x, y, z), 
    axisDirection=(a_x, a_y, a_z), angle=angle)

#----------------------------------------------------------------------------

def Merge_Instance(model,part_new,instance_1,instance_2):
    a = mdb.models[model].rootAssembly
    a.InstanceFromBooleanMerge(name=part_new, instances=(a.instances[instance_1], 
    a.instances[instance_2] ), keepIntersections=ON, originalInstances=SUPPRESS, domain=GEOMETRY)

#----------------------------------------------------------------------------

def Multi_Merge_Instance(model,anz,part_new):
    a = mdb.models[model].rootAssembly
    MyInstance = ()
    MyInstance = MyInstance + (a.instances['Stiffened_Sphere-1'],)
        
    for i in range(2,anz+1,1):
        MyInstance = MyInstance + (a.instances['Stiffened_Sphere-1-rad-'+str(i)],) 
    
    a.InstanceFromBooleanMerge(name=part_new, instances=MyInstance, originalInstances=SUPPRESS, domain=GEOMETRY)

#----------------------------------------------------------------------------

def Delete_Face(model,part,set_face):
    p = mdb.models[model].parts[part]
    f = p.faces
    p.RemoveFaces(faceList = ((set_face),), deleteCells=False)

#----------------------------------------------------------------------------

def Create_Face_Set_ByBoundingSphere(model,part,x1,y1,z1,radius,set_name):
    p = mdb.models[model].parts[part]
    f = p.faces
    faces = f.getByBoundingSphere((x1,y1,z1),radius)
    p.Set(faces=faces, name=set_name)

#----------------------------------------------------------------------------

def Create_Surface_Set_ByBoundingSphere(model,part,x1,y1,z1,radius,set_name):
    p = mdb.models[model].parts[part]
    s = p.faces
    faces = s.getByBoundingSphere((x1,y1,z1),radius)
    p.Surface(side1Faces=faces, name=set_name)

#----------------------------------------------------------------------------


def Create_Boolean_Difference(model,part,set_name_new,set_ref,set_diff):
    a=mdb.models[model].parts[part]
    a.SetByBoolean(name=set_name_new, operation=DIFFERENCE, sets=(a.sets[set_ref], a.sets[set_diff], ))

#----------------------------------------------------------------------------

def Create_Material_Elastic(model,material_name,E,Nu,Y,Rho):
    mdb.models[model].Material(name=material_name)
    mdb.models[model].materials[material_name].Elastic(table=((E, Nu), ))
    mdb.models[model].materials[material_name].Plastic(table=((Y, 0.0), ))
    mdb.models[model].materials[material_name].Density(table=((Rho, ), ))

#----------------------------------------------------------------------------

def Create_Section(model,section_name,material_name,thickness):
    mdb.models[model].HomogeneousShellSection(name=section_name, preIntegrate=OFF, material=material_name, thicknessType=UNIFORM, thickness=thickness, thicknessField='', nodalThicknessField='', idealization=NO_IDEALIZATION, poissonDefinition=DEFAULT, thicknessModulus=None, temperature=GRADIENT, useDensity=OFF, integrationRule=SIMPSON, numIntPts=5)

#----------------------------------------------------------------------------

def Assign_Section(model,set_name,part,section_name):
    p = mdb.models[model].parts[part]
    region = p.sets[set_name]
    p.SectionAssignment(region=region, sectionName=section_name, offset=0.0, offsetType=MIDDLE_SURFACE, offsetField='', thicknessAssignment=FROM_SECTION)
    

# variables

myString = "LBA"


myRadius = 19820
myStiffenerHeight_Circum = 200.0
myStiffenerHeight_Meridian = 200.0
myStiffenerNumber_radial = 5
myStiffenerNumber_meridian = 24
myStiffenerNumber_meridian_large = myStiffenerNumber_meridian/4.0
mySkinThickness = 100
myMeridianThickness = 50
myCircumThickness = 50


MyCutoutRadius = 2500

# The variable mySemiVertexAngle should not exceed 88 !

mySemiVertexAngle = 62.5




myModel = mdb.Model(name=myString)
myPart = "Sphere"
myPart_2 = "Sphere_Stiffener"
myPart_3 = "Stiffened_Sphere"
myPart_4 = "Stiffened_Sphere_final"


# material parameters

myE_Steel = 208000
myNu_Steel = 0.3
myYield_Steel = 550
myDensity_Steel = 7.8E-9

myE_Concrete = 34000
myNu_Concrete = 0.2
myYield_Concrete = 55
myDensity_Concrete = 7.5E-8

Mesh_Size = 250.0



#----------------------------------------------------------------------------

# create model


Create_Part_2D_Stiffened_Sphere(myRadius,mySemiVertexAngle,myStiffenerHeight_Circum,myStiffenerNumber_radial,myPart,myString)
Create_Part_2D_Sphere_Stiffener(myRadius,90.0,myStiffenerHeight_Meridian,myPart_2,myString)

Create_Set_All_Faces(myString,myPart,"Shell_and_circum_face")
Create_Face_Set_ByBoundingSphere(myString,myPart,0,0,0,myRadius,"Sphere_Skin")
Create_Surface_Set_ByBoundingSphere(myString,myPart,0,0,0,myRadius,"Sphere_Skin_Surface")
Create_Boolean_Difference(myString,myPart,"Circum_face","Shell_and_circum_face","Sphere_Skin")
Create_Set_Edge(0.0,myRadius*np.cos(mySemiVertexAngle*np.pi/180.0),myRadius*np.sin(mySemiVertexAngle*np.pi/180.0),myString,myPart,"Set-Bottom-Edge")

Create_Set_All_Faces(myString,myPart_2,"Stiffener_face")

Create_Assembly(myString,myPart,"Sphere")
Create_Assembly(myString,myPart_2,"Sphere_Stiffener")

Merge_Instance(myString,myPart_3,myPart_2,myPart)

mySetFace_1 = Create_Set_Face(myRadius,0.0,0.0,myString,myPart_3,"Stiffener_Part_Bottom")

Delete_Face(myString,myPart_3,mySetFace_1)

Radial_Pattern_Instance(myString,'Stiffened_Sphere-1',0,0,0,0,1,0,myStiffenerNumber_meridian,360)

Multi_Merge_Instance(myString,myStiffenerNumber_meridian,myPart_4)

myID_1 = Create_Datum_Plane_by_Principal(XZPLANE,myPart_4,myString,0.0)
myEdge = Create_Set_Edge(0.0,myRadius,0.0,myString,myPart_4,"Set-Top-Edge_sphere")

CreateCutout(myString,myPart_4,MyCutoutRadius,myID_1,myEdge,0.0,0.0,0.0)

Create_Material_Elastic(myString,"Steel",myE_Steel,myNu_Steel,myYield_Steel,myDensity_Steel)
Create_Material_Elastic(myString,"Concrete",myE_Concrete,myNu_Concrete,myYield_Concrete,myDensity_Concrete)


Create_Section(myString,"Concrete_Section","Concrete",mySkinThickness)
Create_Section(myString,"Steel_Section_Meridian","Steel",myMeridianThickness)
Create_Section(myString,"Steel_Section_Circum","Steel",myCircumThickness)

Assign_Section(myString,"Sphere_Skin",myPart_4,"Concrete_Section")
Assign_Section(myString,"Stiffener_face",myPart_4,"Steel_Section_Meridian")
Assign_Section(myString,"Circum_face",myPart_4,"Steel_Section_Circum")


Create_LBA_Step(myString)

Create_Boundary_Condition_by_Instance(myString,"Stiffened_Sphere_final-1","Set-Bottom-Edge","BC_Bottom","Initial",SET,SET,SET,SET,SET,SET)
Create_Pressure_Load(myString, "Stiffened_Sphere_final-1", "Pressure", "Step-1", "Sphere_Skin_Surface", 1)


Create_Mesh(myString,myPart_4,Mesh_Size)


CreateJob(myString,"LBA",1)

# SubmitJob("LBA")

#--------------------