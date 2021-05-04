import numpy as np

Rho  = 1000
RhoG = Rho  * 9.81

File = open("DDBoat2.obj","r")
lignes = File.readlines()

"""
Body = np.array([ [-1, 1,-1, 1,-1, 1,-1, 1] ,
		   [-1,-1, 1, 1,-1,-1, 1, 1] ,
		   [-1,-1,-1,-1, 1, 1, 1, 1] ])
liste = [[0,1,2] ,[3,1,2] ,[0,1,5] ,[0,5,4] ,[0,4,2] ,[4,6,2] ,[2,3,7] ,[2,7,6] ,[1,3,7] ,[1,7,5] ,[5,6,7] ,[5,6,4] ]
"""

tempo = []
liste = []

Dist = 0.75

for i in range(len(lignes)):
	ligne=lignes[i].split(" ")
	if ligne[0]=="v":
		tempo.append([float(ligne[1]),float(ligne[2]),float(ligne[3])])
	if ligne[0]=="f":
		liste.append([int(ligne[1].split("/")[0])-1,int(ligne[2].split("/")[0])-1,int(ligne[3].split("/")[0])-1])

Body = np.array(tempo).T






def normaliz(X):
	value = np.linalg.norm(X)
	if value==0:
		return 0*X
	return X/value

def sign(value):
	if value>=0:
		return 1
	return -1

listeN = []
for i in range(len(liste)):
	normal = np.cross(Body[:,liste[i][1]]-Body[:,liste[i][0]],Body[:,liste[i][2]]-Body[:,liste[i][0]])
	normal = normaliz(normal)*sign(np.vdot(normal,Body[:,liste[i][0]]))
	listeN.append(normal)


def force_dynamique(centre,PointApl,normal,vitesse,omega,S):
	speed = (vitesse + np.cross(omega,centre-PointApl) ) * -1
	vitN = np.vdot(normal,speed)*normal
	vitT = speed - vitN
	ForceN = vitN*Rho*0.5*np.linalg.norm(vitN)*S
	vT = np.linalg.norm(vitT)
	Re = vT*Dist/(10**-3)
	if Re > 0:
		Cf = 1.328/Re**0.5
		ForceT = vitT*vT*0.5*Rho*S*Cf
	else:
		ForceT = np.zeros(3)
	return ForceN + ForceT
	
	

def triangle_PointeHaut(triangle,PointApl,normal,vitesse,omega):
	#print(triangle)
	b = np.linalg.norm(triangle[1]-triangle[2])
	if b==0:
		return np.array([0,0,0]),np.array([0,0,0])
	copy = []
	for i in range(3):
		copy.append(np.array(triangle[i]))
		copy[-1][2] = 0
	h = triangle[0][2] - triangle[2][2]
	d = abs( np.vdot(copy[0]-copy[1], normaliz( np.cross(copy[2]-copy[1], np.array([0,0,1])) ) ) )
	z0= triangle[2][2]
	S = 0.5*b*d
	centre = (2/3)*((triangle[1]+triangle[2])/2 - triangle[0]) + triangle[0]
	FV = (S*h/3)*RhoG 
	FS = S*RhoG*abs(z0)
	Force = - (FS-FV) * sign(np.vdot(normal,np.array([0,0,1])))* np.array([0,0,1]) + force_dynamique(centre,PointApl,normal,vitesse,omega,0.5*b*(h**2+d**2)**0.5)
	M = np.cross(centre-PointApl,Force)
	return Force,M
	
def triangle_PointeBas(triangle,PointApl,normal,vitesse,omega):
	return triangle_PointeHaut([triangle[2],triangle[1],triangle[0]],PointApl,normal,vitesse,omega)
	
	


def divise_Triangle(triangle,PointApl,normal,vitesse,omega):
	if triangle[0][2]-triangle[2][2] == 0:
		return triangle_PointeHaut(triangle,PointApl,normal,vitesse,omega)
	coeff = (triangle[1][2]-triangle[2][2])/(triangle[0][2] - triangle[2][2])
	PointAdd = (triangle[0] - triangle[2])*coeff + triangle[2]
	listeUp = [triangle[0],PointAdd,triangle[1]]
	listeBa = [PointAdd,triangle[1],triangle[2]]
	F1,M1 = triangle_PointeHaut(listeUp,PointApl,normal,vitesse,omega)
	F2,M2 = triangle_PointeBas(listeBa,PointApl,normal,vitesse,omega)
	return F1+F2 , M1+M2

def found_Niv_Eau(PH,PB):
	if PH[2]-PB[2]==0:
		return PH
	coeff = (0-PB[2])/(PH[2]-PB[2])
	PointAdd = (PH - PB)*coeff + PB
	return PointAdd
	

def Calcul_Triangle(triangle1,PointApl,normal,vitesse,omega):
	#print(triangle)
	triangle = []
	for i in range(3):
		triangle.append(np.array(triangle1[i]))
	for i in range(2):
		for k in range(2):
			if triangle[k][2] < triangle[k+1][2]:
				triangle[k],triangle[k+1] = triangle[k+1],triangle[k]
	if triangle[2][2] > 0:
		return np.array([0,0,0]),np.array([0,0,0])
	elif triangle[1][2] > 0: # 2 points au dessus
		P1 = found_Niv_Eau(triangle[0],triangle[2])
		P2 = found_Niv_Eau(triangle[1],triangle[2])
		listeBa = [ P1, P2, triangle[2] ]
		return triangle_PointeBas(listeBa,PointApl,normal,vitesse,omega)
	elif triangle[0][2] > 0: # 1 point  au dessus
		P1 = found_Niv_Eau(triangle[0],triangle[1])
		P2 = found_Niv_Eau(triangle[0],triangle[2])
		liste1 = [ P1, P2, triangle[2] ]
		liste2 = [ triangle[1], P1, triangle[2] ]
		F1,M1 = triangle_PointeBas(liste1,PointApl,normal,vitesse,omega)
		F2,M2 = divise_Triangle(liste2,PointApl,normal,vitesse,omega)
		return F1+F2 , M1+M2
	else: #  complet
		return divise_Triangle(triangle,PointApl,normal,vitesse,omega)

def Calcul(R,pos,vitesse,omega):
	F = np.zeros(3)
	M = np.zeros(3)
	Corps = R@Body
	for i in range(3):
		Corps[i,:] += pos[i]
	for i in range(len(liste)):
		dF,dM = Calcul_Triangle([Corps[:,liste[i][0]],Corps[:,liste[i][1]],Corps[:,liste[i][2]]],pos,R@listeN[i],vitesse,omega)#waring v,omega ou vr,omegar
		F = F + dF
		M = M + dM
	return F,M
	
	
