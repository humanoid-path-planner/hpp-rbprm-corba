SET(${PROJECT_NAME}_PYTHON_TESTS
  test_talos_walk_contacts
  test_talos_walk_path
  )

FOREACH(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
  ADD_PYTHON_UNIT_TEST("py-${TEST}" "tests/python/${TEST}.py")
ENDFOREACH(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
