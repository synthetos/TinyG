/*
 * config_master.h - the persisted rows of master builds 440.20 and 440.21 (N92)
 *
 * Generated from master d0418390 firmware/tinyg/config_app.c (preprocessed with the master Makefile flags;
 * 396 rows, matching the 9504-byte cfgArray of the built ELF; 440.20 at 42688b6c is identical). Master
 * persists every F_INITIALIZE row as a 4-byte float at index * 4 from EEPROM address 0; record 0 holds the
 * build number. Row i is the i-th string: its token if master loaded it at boot, else empty. Master's
 * status report slots (se00-) are not among them. The separate full token map below preserves saved selections.
 * config_init() uses this to migrate a board upgraded from master. Do not edit by hand.
 */
#ifndef CONFIG_MASTER_H_ONCE
#define CONFIG_MASTER_H_ONCE

#define MASTER_NV_ROWS 396

// one NUL-terminated string per master row, in index order (\000: a token may start with a digit)
static const char master_tokens[] PROGMEM =
	"\000fv\000hp\000hv\000\000n\000line\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000"
	"\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000"
	"\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\0001ma\000"
	"1sa\0001tr\0001mi\0001po\0001pm\0002ma\0002sa\0002tr\0002mi\0002po\0002pm\0003ma\0003sa\0003tr\0003mi\000"
	"3po\0003pm\0004ma\0004sa\0004tr\0004mi\0004po\0004pm\000xam\000xvm\000xfr\000xtn\000xtm\000xjm\000xjh\000"
	"xjd\000xsn\000xsx\000xsv\000xlv\000xlb\000xzb\000yam\000yvm\000yfr\000ytn\000ytm\000yjm\000yjh\000yjd\000"
	"ysn\000ysx\000ysv\000ylv\000ylb\000yzb\000zam\000zvm\000zfr\000ztn\000ztm\000zjm\000zjh\000zjd\000zsn\000"
	"zsx\000zsv\000zlv\000zlb\000zzb\000aam\000avm\000afr\000atn\000atm\000ajm\000ajh\000ajd\000ara\000asn\000"
	"asx\000asv\000alv\000alb\000azb\000bam\000bvm\000bfr\000btn\000btm\000bjm\000bjd\000bra\000cam\000cvm\000"
	"cfr\000ctn\000ctm\000cjm\000cjd\000cra\000p1frq\000p1csl\000p1csh\000p1cpl\000p1cph\000p1wsl\000p1wsh\000"
	"p1wpl\000p1wph\000p1pof\000g54x\000g54y\000g54z\000g54a\000g54b\000g54c\000g55x\000g55y\000g55z\000g55a\000"
	"g55b\000g55c\000g56x\000g56y\000g56z\000g56a\000g56b\000g56c\000g57x\000g57y\000g57z\000g57a\000g57b\000"
	"g57c\000g58x\000g58y\000g58z\000g58a\000g58b\000g58c\000g59x\000g59y\000g59z\000g59a\000g59b\000g59c\000"
	"g92x\000g92y\000g92z\000g92a\000g92b\000g92c\000g28x\000g28y\000g28z\000g28a\000g28b\000g28c\000g30x\000"
	"g30y\000g30z\000g30a\000g30b\000g30c\000\000\000\000\000ja\000ct\000sl\000st\000mt\000\000\000ej\000jv\000"
	"js\000tv\000qv\000sv\000si\000ec\000ee\000ex\000\000net\000gpl\000gun\000gco\000gpa\000gdi\000\000ma\000"
	"fd\000uda0\000uda1\000uda2\000uda3\000udb0\000udb1\000udb2\000udb3\000udc0\000udc1\000udc2\000udc3\000"
	"udd0\000udd1\000udd2\000udd3\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000"
	"\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000"
	"\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000"
	"\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000"
	"\000\000\000\000\000\000\000\000\000\000\000\000";

// The same preprocessed master table, including readouts, for saved status-list indexes.
#define MASTER_SR_FIRST 321
static const char master_report_tokens[] PROGMEM =
	"fb\000fv\000hp\000hv\000id\000n\000line\000vel\000feed\000stat\000macs\000cycs\000mots\000hold\000"
	"unit\000coor\000momo\000plan\000path\000dist\000frmo\000tool\000mpox\000mpoy\000mpoz\000mpoa\000"
	"mpob\000mpoc\000posx\000posy\000posz\000posa\000posb\000posc\000ofsx\000ofsy\000ofsz\000ofsa\000"
	"ofsb\000ofsc\000home\000homx\000homy\000homz\000homa\000homb\000homc\000prbe\000prbx\000prby\000"
	"prbz\000prba\000prbb\000prbc\000jogx\000jogy\000jogz\000joga\000pwr1\000pwr2\000pwr3\000pwr4\000"
	"sr\000qr\000qi\000qo\000er\000qf\000rx\000msg\000clear\000test\000defa\000boot\000help\000h\000"
	"1ma\0001sa\0001tr\0001mi\0001po\0001pm\0002ma\0002sa\0002tr\0002mi\0002po\0002pm\0003ma\0003sa\000"
	"3tr\0003mi\0003po\0003pm\0004ma\0004sa\0004tr\0004mi\0004po\0004pm\000xam\000xvm\000xfr\000xtn\000"
	"xtm\000xjm\000xjh\000xjd\000xsn\000xsx\000xsv\000xlv\000xlb\000xzb\000yam\000yvm\000yfr\000ytn\000"
	"ytm\000yjm\000yjh\000yjd\000ysn\000ysx\000ysv\000ylv\000ylb\000yzb\000zam\000zvm\000zfr\000ztn\000"
	"ztm\000zjm\000zjh\000zjd\000zsn\000zsx\000zsv\000zlv\000zlb\000zzb\000aam\000avm\000afr\000atn\000"
	"atm\000ajm\000ajh\000ajd\000ara\000asn\000asx\000asv\000alv\000alb\000azb\000bam\000bvm\000bfr\000"
	"btn\000btm\000bjm\000bjd\000bra\000cam\000cvm\000cfr\000ctn\000ctm\000cjm\000cjd\000cra\000p1frq\000"
	"p1csl\000p1csh\000p1cpl\000p1cph\000p1wsl\000p1wsh\000p1wpl\000p1wph\000p1pof\000g54x\000g54y\000"
	"g54z\000g54a\000g54b\000g54c\000g55x\000g55y\000g55z\000g55a\000g55b\000g55c\000g56x\000g56y\000"
	"g56z\000g56a\000g56b\000g56c\000g57x\000g57y\000g57z\000g57a\000g57b\000g57c\000g58x\000g58y\000"
	"g58z\000g58a\000g58b\000g58c\000g59x\000g59y\000g59z\000g59a\000g59b\000g59c\000g92x\000g92y\000"
	"g92z\000g92a\000g92b\000g92c\000g28x\000g28y\000g28z\000g28a\000g28b\000g28c\000g30x\000g30y\000"
	"g30z\000g30a\000g30b\000g30c\000jida\000jidb\000jidc\000jidd\000ja\000ct\000sl\000st\000mt\000me\000"
	"md\000ej\000jv\000js\000tv\000qv\000sv\000si\000ec\000ee\000ex\000baud\000net\000gpl\000gun\000"
	"gco\000gpa\000gdi\000gc\000ma\000fd\000uda0\000uda1\000uda2\000uda3\000udb0\000udb1\000udb2\000"
	"udb3\000udc0\000udc1\000udc2\000udc3\000udd0\000udd1\000udd2\000udd3\000_tex\000_tey\000_tez\000"
	"_tea\000_teb\000_tec\000_trx\000_try\000_trz\000_tra\000_trb\000_trc\000_ts1\000_ps1\000_cs1\000"
	"_es1\000_xs1\000_fe1\000_ts2\000_ps2\000_cs2\000_es2\000_xs2\000_fe2\000_ts3\000_ps3\000_cs3\000"
	"_es3\000_xs3\000_fe3\000_ts4\000_ps4\000_cs4\000_es4\000_xs4\000_fe4\000_dam\000se00\000se01\000"
	"se02\000se03\000se04\000se05\000se06\000se07\000se08\000se09\000se10\000se11\000se12\000se13\000"
	"se14\000se15\000se16\000se17\000se18\000se19\000se20\000se21\000se22\000se23\000se24\000se25\000"
	"se26\000se27\000se28\000se29\000sys\000p1\0001\0002\0003\0004\000x\000y\000z\000a\000b\000c\000"
	"g54\000g55\000g56\000g57\000g58\000g59\000g92\000g28\000g30\000mpo\000pos\000ofs\000hom\000prb\000"
	"pwr\000jog\000jid\000uda\000udb\000udc\000udd\000_te\000_tr\000_ts\000_ps\000_cs\000_es\000_xs\000"
	"_fe\000m\000q\000o\000$\000";

#endif // CONFIG_MASTER_H_ONCE
