// The '~' command channel must claim ONLY its own bytes and never swallow a byte
// of OpenFIRE's protocol, and every reply must go to the installed output sink
// rather than printf.
#include "aim_runtime.h"
#include <stdio.h>
#include <string.h>
#include <initializer_list>
static int fails=0;
static void ck(bool ok,const char*m){printf("  [%s] %s\n",ok?"PASS":"FAIL",m); if(!ok)fails++;}
static char g_extra[128]; static bool g_extra_hit=false;
// capture what the module tries to REPLY, to prove it uses the sink and not printf
static char g_out[2048]; static size_t g_outn=0;
static void sink(const char* s){ size_t n=strlen(s); if(g_outn+n<sizeof(g_out)){ memcpy(g_out+g_outn,s,n); g_outn+=n; g_out[g_outn]=0; } }
// Claim ONLY camera lines, exactly as the real handler does -- a handler that
// claimed everything would make the "unknown command" assertion untestable.
static bool extra(const char* l){
    if (strncmp(l,"cam=",4)) return false;
    snprintf(g_extra,sizeof(g_extra),"%s",l); g_extra_hit=true; return true;
}

// feed a string; return what was LEFT for OpenFIRE's parser
static const char* feed(const char* in, char* left, size_t n)
{
    size_t j=0;
    for (const char* p=in; *p; ++p) {
        if (!aim_serial_rx(*p)) { if (j+1<n) left[j++]=*p; }
    }
    left[j]=0; return left;
}
int main(){
    aim_runtime_begin();
    aim_serial_set_extra(extra);
    aim_set_out(sink);
    char left[256];

    printf("bytes left for OpenFIRE's parser:\n");
    ck(!strcmp(feed("~aimfilt=1.0,15\n",left,sizeof(left)), ""), "our line is fully consumed");
    // their real commands must pass through untouched
    for (const char* their : {"S6\n","M0\n","XI3\n","F0\n","E\n"}) {
        char l[64]; feed(their,l,sizeof(l));
        if(strcmp(l,their)) { printf("  [FAIL] mangled %s -> %s\n",their,l); fails++; }
    }
    ck(true, "OpenFIRE commands S/M/X/F/E pass through byte-for-byte");

    // interleaved: their command, ours, their command
    ck(!strcmp(feed("S6\n~aimcap=1\nM0\n",left,sizeof(left)), "S6\nM0\n"),
       "interleaved streams separate cleanly");

    printf("\ndispatch:\n");
    g_extra_hit=false;
    feed("~cam=thr:60,aec:40\n",left,sizeof(left));
    ck(g_extra_hit && !strcmp(g_extra,"cam=thr:60,aec:40"),
       "camera line reaches the extra handler with '~' stripped");
    feed("~aimcal=0.5046,0.2988,0.3513,1.2003,4.8,-2.12\n",left,sizeof(left));
    ck(aim_runtime_active(), "calibration installs over the '~' channel");
    feed("~aimfilt=2.0,20\n",left,sizeof(left));
    ck(aim_filter_min_cutoff()==2.0f && aim_filter_beta()==20.0f, "filter set over '~'");

    printf("\nrobustness:\n");
    // an overlong line must resync, not wrap and corrupt
    char big[300]; memset(big,'x',sizeof(big)); big[0]='~'; big[sizeof(big)-2]='\n'; big[sizeof(big)-1]=0;
    feed(big,left,sizeof(left));
    ck(!strcmp(feed("S6\n",left,sizeof(left)),"S6\n"),
       "an overlong '~' line resyncs and does not eat the next command");
    // a bare CR terminator
    g_extra_hit=false; feed("~cam=thr:50\r",left,sizeof(left));
    ck(g_extra_hit, "CR terminates a line as well as LF");

    // Replies must go to the installed sink: printf() answers down UART0 even
    // when the query arrived over native USB.
    printf("\nboth dialects accepted (the two transports must agree):\n");
    ck(aim_runtime_command("aimfilt=1.0,15"),  "bare 'aimfilt=' (UART0 dialect)");
    ck(aim_runtime_command("~aimfilt=2.0,20"), "'~aimfilt=' (native-USB dialect)");
    ck(aim_filter_min_cutoff()==2.0f, "the '~' form actually applied");

    printf("\nreplies go to the sink, not printf:\n");
    g_outn=0; g_out[0]=0;
    feed("~aimcal?\n",left,sizeof(left));
    ck(strstr(g_out,"AIM:")!=0, "'~aimcal?' reply reached the sink");
    g_outn=0; g_out[0]=0;
    feed("~aimfilt?\n",left,sizeof(left));
    ck(strstr(g_out,"filter")!=0, "'~aimfilt?' reply reached the sink");
    g_outn=0; g_out[0]=0;
    feed("~aimcap=1\n",left,sizeof(left));
    ck(strstr(g_out,"trigger markers")!=0, "'~aimcap=1' reply reached the sink");
    g_outn=0; g_out[0]=0;
    aim_runtime_trigger_tick(true);
    ck(strstr(g_out,"T,")!=0, "trigger MARKER goes to the sink too");
    g_outn=0; g_out[0]=0;
    feed("~aimnonsense\n",left,sizeof(left));
    ck(strstr(g_out,"unknown")!=0, "even the error reply reaches the sink");

    printf("\n%s (%d failures)\n", fails?"FAILURES":"ALL PASS", fails);
    return fails?1:0;
}
