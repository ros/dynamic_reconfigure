class ${upper}
{
  public:
    ${upper}()
    {
      state = true;
      name = "${name}";
    }
    bool state;
    std::string name;

    ${subgroups}
}${lower};
