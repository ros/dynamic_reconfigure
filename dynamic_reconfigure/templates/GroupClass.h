class ${upper}
{
  public:
    ${upper}()
    {
      state = true;
      name = "${name}";
    }

    void setParams(${configname}Config &config, const std::vector<AbstractParamDescriptionConstPtr> params)
    {
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = params.begin(); i != params.end(); i++)
      {
        boost::any val;
        (*i)->getValue(config, val);

${setters}
      }
    }

    ${params}

    bool state;
    std::string name;

    ${subgroups}
}${lower};
