void do_nothing(){
#ifndef THIS_IS_MY_EXPORTED_DEFINITION
#error "THIS_IS_MY_EXPORTED_DEFINITION is not set"
#endif
#if THIS_IS_MY_OTHER_EXPORTED_DEFINITION != 1
#error "THIS_IS_MY_OTHER_EXPORTED_DEFINITION is not 1"
#endif
}
