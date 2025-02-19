#cylinder inertia about center

# drive link
mass = 0.00016893
radius = 0.00075
height = 0.0383

#humerus
# mass = 0.00016893
# radius = 0.00075
# height = 0.04

#radius
# mass = 0.00041232
# radius = 0.00075
# # height = 0.1
# height = 0.05
I_xx = 0.25*mass*radius**2 + (1.0/12.0)*mass*height**2
I_yy = 0.25*mass*radius**2 + (1.0/12.0)*mass*height**2
I_zz = 0.5*mass*radius**2
I_xy = 0.0
I_xz = 0.0
I_yz = 0.0

print("<ixx>"+str(I_xx)+"</ixx>")
print("<iyy>"+str(I_yy)+"</iyy>")
print("<izz>"+str(I_zz)+"</izz>")
print("<ixy>"+str(I_xy)+"</ixy>")
print("<ixz>"+str(I_xz)+"</ixz>")
print("<iyz>"+str(I_yz)+"</iyz>")


# drive link
length = width = radius * 2

I_xx = 1/12 * mass * (width**2 + length**2)
I_yy = 1/12 * mass * (height**2 + length**2)
I_zz = 1/12 * mass * (height**2 + width**2)

print("Box")
print("<ixx>"+str(I_xx)+"</ixx>")
print("<iyy>"+str(I_yy)+"</iyy>")
print("<izz>"+str(I_zz)+"</izz>")
print("<ixy>"+str(I_xy)+"</ixy>")
print("<ixz>"+str(I_xz)+"</ixz>")
print("<iyz>"+str(I_yz)+"</iyz>")
