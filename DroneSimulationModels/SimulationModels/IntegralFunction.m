function IntegralNew = IntegralFunction(PrevIntegralValue,Error, SimParams)
    ErrorAntiWindup = Error-0.001*PrevIntegralValue;
    IntegralNew = PrevIntegralValue+ErrorAntiWindup*SimParams.dt;
end