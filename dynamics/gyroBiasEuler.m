function btp1 = gyroBiasEuler(b,random_walk,dt)
    btp1 = b + dt*gyroBias(0,b,random_walk);
end

