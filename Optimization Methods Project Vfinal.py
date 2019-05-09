
# coding: utf-8

# # CARGO OPERATIONS OF AIR EXPRESS

# In[1]:


# Import data and packages


from gurobi import *

cargo = [[[0 for t in range(5)]for i in range(3)]for j in range(3)]

cargo[0][1][0]=100
cargo[0][2][0]=50
cargo[1][0][0]=25
cargo[1][2][0]=25
cargo[2][0][0]=40
cargo[2][1][0]=400

cargo[0][1][1]=200
cargo[0][2][1]=50
cargo[1][0][1]=25
cargo[1][2][1]=25
cargo[2][0][1]=40
cargo[2][1][1]=200

cargo[0][1][2]=100
cargo[0][2][2]=50
cargo[1][0][2]=25
cargo[1][2][2]=25
cargo[2][0][2]=40
cargo[2][1][2]=300

cargo[0][1][3]=400
cargo[0][2][3]=50
cargo[1][0][3]=25
cargo[1][2][3]=25
cargo[2][0][3]=40
cargo[2][1][3]=200

cargo[0][1][4]=300
cargo[0][2][4]=50
cargo[1][0][4]=25
cargo[1][2][4]=25
cargo[2][0][4]=40
cargo[2][1][4]=400

# define model

model = Model("Express Air")


# # Parameters

# In[2]:


COST_AB = 7
COST_BC = 5
COST_AC = 3
COST_HOLD = 10
FLEET = 1200


# # Decision Variables

# In[3]:


#full plane movements

f = [[[0 for t in range(5)]for j in range(3)]for i in range(3)]

for t in range(5):
    for j in range(3):
        for i in range(3):
            f[i][j][t]=model.addVar(vtype = GRB.INTEGER, 
                                    name = "full_planes_"+ str(i)+ "_" + str(j) + "_" + str(t))

model.update()

#empty plane movements

e = [[[0 for t in range(5)]for j in range(3)]for i in range(3)]

for t in range(5):
    for j in range(3):
        for i in range(3):
            e[i][j][t]=model.addVar(vtype = GRB.INTEGER, 
                                    name = "empty_planes_" + str(i)+ "_" + str(j) + "_" + str(t))

model.update()

#cargos remaining in airports

h = [[[0 for t in range(5)]for j in range(3)]for i in range(3)]

for t in range(5):
    for j in range(3):
        for i in range(3):
            h[i][j][t]=model.addVar(vtype = GRB.INTEGER, 
                                    name = "amount_staying_in_" + str(i)+ "_going_to_" + str(j) + "_on_period_" + str(t))

model.update()


# # Objective Function

# In[4]:


objective_function = LinExpr()

for t in range(5):
    objective_function+= COST_AB*(e[0][1][t]+e[1][0][t])+COST_BC*(e[1][2][t]+e[2][1][t])+COST_AC*(e[0][2][t]+e[2][0][t])+COST_HOLD*(h[0][1][t]+h[0][2][t]+h[1][0][t]+h[1][2][t]+h[2][0][t]+h[2][1][t])

model.setObjective(objective_function,GRB.MINIMIZE)

model.update()


# # Constraints

# In[5]:


# no full planes from node to node

for t in range(5):
    for i in range(3):
        for j in range(3):
            if i==j:
                model.addConstr(lhs = f[i][j][t], 
                                sense = GRB.EQUAL, 
                                rhs = 0,
                                name = "no_f_" + str(i) + "_" + str(j) + "_" + str(t))
  
# no holding from node to node

for t in range(5):
    for i in range(3):
        for j in range(3):
            if i==j:
                model.addConstr(lhs = h[i][j][t], 
                                sense = GRB.EQUAL, 
                                rhs = 0,
                                name = "no_h_" + str(i) + "_" + str(j) + "_" + str(t))
   

# 1200 planes constraint

for t in range(5):
    lhs = LinExpr()
    for i in range(3):
        for j in range(3):
            lhs+=f[i][j][t]+e[i][j][t]
    model.addConstr(lhs = lhs, 
                    sense = GRB.EQUAL, 
                    rhs = FLEET,
                    name = "1200_planes_at_" + str(t))      

    
# cargo entering node = cargo leaving node

for t in range(5):
    for i in range(3):
        for j in range(3): 
            lhs = f[i][j][t]
            rhs = h[i][j][t-1] + cargo[i][j][t] - h[i][j][t]
            model.addConstr(lhs = lhs, 
                            sense = GRB.EQUAL, 
                            rhs = rhs,
                            name = "available_cargo" ) 
            
model.update()

# node supply requirements

for t in range(5):
    for i in range(3):
        for j in range(3):
            model.addConstr(lhs = f[i][j][t], 
                            sense = GRB.LESS_EQUAL, 
                            rhs = cargo[i][j][t] + h[i][j][t-1],
                            name = "max_amount_of_f_" + str(i) + "_" + str(j) + "_" + str(t)) 
                               

# total cargo from i to j

for i in range(3):
    for j in range(3):
        lhs=LinExpr()
        rhs = LinExpr()
        for t in range(5):
            lhs+=f[i][j][t]
            rhs+=cargo[i][j][t]
        model.addConstr(lhs = lhs, 
                        sense = GRB.EQUAL,
                        rhs = rhs,
                        name = "total_cargo_from_" + str(i) + " to " + str(j))        
            

# constrain amount of available planes at each airport at each time

for t in range(5):
    for i in range(3):
        lhs=LinExpr()
        rhs=LinExpr()
        for j in range(3): 
            lhs+=f[i][j][t]+e[i][j][t]
            rhs+=f[j][i][t-1]+e[j][i][t-1]
        model.addConstr(lhs = lhs, 
                        sense = GRB.EQUAL, 
                        rhs = rhs,
                        name = "available_planes_at_" + str(i)) 
        

            
model.update()


# # OPTIMIZE

# In[6]:


model.write("ExpressAirV4.lp")

model.optimize()
print("\nOptimal Objective: " + str(model.ObjVal)) 

#extract decision variables
print("\nOptimal Solution:") 
allVars = model.getVars()
for var in allVars:
    if var.x != 0:
        print(var.varName + " " + str(var.x))

