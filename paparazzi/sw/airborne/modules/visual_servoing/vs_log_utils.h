#ifndef VS_LOG_UTILS_H
#define VS_LOG_UTILS_H
/* Linux/NPS logging helpers. Wall time is used only for names/profiling. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/stat.h>
#include <errno.h>
static inline uint64_t vs_profile_us(void)
{
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC,&t);
  return (uint64_t)t.tv_sec*1000000ULL+t.tv_nsec/1000U;
}
static inline int vs_mkdirs(const char *path)
{
  char tmp[1024];
  if (!path || !*path || strlen(path)>=sizeof(tmp)) { return -1; }
  strcpy(tmp,path);
  for (char *c=tmp+1;*c;c++) {
    if (*c=='/') {
      *c=0;
      if (mkdir(tmp,0775) && errno!=EEXIST) { return -1; }
      *c='/';
    }
  }
  return mkdir(tmp,0775) && errno!=EEXIST ? -1 : 0;
}
static inline const char *vs_run_id(void)
{
  static char id[128];
  if (!id[0]) {
    const char *env=getenv("B54_RUN_ID");
    if (env && *env) {
      size_t j=0;
      for (;env[j] && j<sizeof(id)-1;j++) {
        char c=env[j];
        id[j]=((c>='a'&&c<='z')||(c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='-'||c=='_') ? c : '_';
      }
      id[j]=0;
    } else {
      struct timespec t;
      clock_gettime(CLOCK_REALTIME,&t);
      snprintf(id,sizeof(id),"%lld_%09ld_%ld",(long long)t.tv_sec,t.tv_nsec,(long)getpid());
      setenv("B54_RUN_ID",id,1);
    }
  }
  return id;
}
#endif
