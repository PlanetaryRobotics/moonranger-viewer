#ifndef __DB_H__
#define __DB_H__

#ifdef USE_MYSQL

#include <rasm_common_types.h>
#include <rasm_pose.h>
//#include <tmap_full.h>

void insert_db_record(const RASM::point3d* cloud,
		      const unsigned int num_cloud_points,
		      const RASM::pose& cloud_pose,
		      const unsigned int sequence_number);

void close_db();

unsigned int num_rows_returned_by_query(char* query,
					unsigned int query_length);

void send_sql_query(const char* query);

// TBD: add something like this function back in, but implement TMAP-centric 
// architecture first
/* bool get_tmap(char* query, */
/* 	      TMAP::tmap& new_data, */
/* 	      RASM::pose& cloud_pose); */

#endif // USE_MYSQL

#endif /* __DB_H__ */
