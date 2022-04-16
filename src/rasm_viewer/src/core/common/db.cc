
#ifdef USE_MYSQL

#include <stdio.h>
#include <stdlib.h>
#include <dlfcn.h>
#include <errno.h>
#include <mysql/mysql.h>
#include "util.h"
#include "db.h"

static const char* g_db_host_name = "localhost";
static const char* g_db_user_name = "root";
static const char* g_db_passwd = "";
static const char* g_db_name = "mesh";

static MYSQL* g_mysql = NULL;


void connect_to_db()
{
  g_mysql = mysql_init(NULL);
  
  assert(g_mysql != NULL);
  
  assert(g_mysql == mysql_real_connect(g_mysql,
				       g_db_host_name, 
				       g_db_user_name, 
				       g_db_passwd, 
				       g_db_name, 
				       0, NULL, 0));
  
  printf("database CONNECTED\n");
}


void insert_db_record(const RASM::point3d* cloud,
		      const unsigned int num_cloud_points,
		      const RASM::pose& cloud_pose,
		      const unsigned int sequence_number)
{
  assert(NULL != cloud);
  assert(num_cloud_points >= 3);
  assert(cloud_pose.isValid());

  /*
   * check to see if we need to connect to the db
   */
  if(NULL == g_mysql) connect_to_db();

  /*
   * TBD: find something faster than running through clouds and replacing
   * escape chars each time....
   */
  int bytes_in_cloud = num_cloud_points*sizeof(RASM::point3d);
  char* buffer = (char*)malloc(bytes_in_cloud*2+1); // TBD: maybe speed this up with realloc?
  assert(NULL != buffer);
  mysql_real_escape_string(g_mysql, buffer, (char*)cloud, bytes_in_cloud);

  /*
   * Now construct INSERT query...must be a faster way to do this too!
   */
  char* query = (char*)malloc(bytes_in_cloud*2 + 1 + 10000/*for other chars*/);

  RASM::point3d pos = cloud_pose.getPosition();

  // TBD: use more robust snprintf()
  int len = sprintf(query,
		    "INSERT INTO meshes(id, rover_x, rover_y, rover_z, rover_roll, rover_pitch, rover_yaw, vertices, timestamp) VALUES('%d', '%f', '%f', '%f', '%f', '%f', '%f', '%s', '%f')",
		    sequence_number,
		    RASM_TO_METERS(pos.X()), RASM_TO_METERS(pos.Y()), RASM_TO_METERS(pos.Z()),
		    cloud_pose.getOrientation().roll, cloud_pose.getOrientation().pitch, cloud_pose.getOrientation().yaw,
		    buffer,
		    cloud_pose.getTime());
  assert(len > 0);

  printf("query returned %d (%d %s)\n", mysql_real_query(g_mysql, query, len), errno, strerror(errno));

  free(buffer); // TBD: keep these in sync with above calls to {re,m}alloc
  free(query);
}


void close_db()
{
  mysql_close(g_mysql);
  g_mysql = NULL;
}


unsigned int num_rows_returned_by_query(char* query,
					unsigned int query_length)
{
  assert(NULL != query);

  /*
   * check to see if we need to connect to the db
   */
  if(NULL == g_mysql) connect_to_db();

  int status = mysql_real_query(g_mysql, query, query_length);
  if(0 != status) return 0;

  MYSQL_RES* result = mysql_store_result(g_mysql);
  if(NULL == result) return 0;

  MYSQL_ROW row = mysql_fetch_row(result);
  unsigned int num_rows = 0;
  while(row != NULL)
    {
      num_rows++;
      row = mysql_fetch_row(result);
    }

  mysql_free_result(result);

  return num_rows;
}

// bool get_tmap(char* query,
// 	      TMAP::tmap& new_data,
// 	      RASM::pose& cloud_pose)
// {
//   assert(NULL != query);

//   if(NULL == g_mysql) connect_to_db();

//   int status = mysql_real_query(g_mysql, query, strlen(query));
//   if(0 != status) return false;

//   MYSQL_RES* result = mysql_store_result(g_mysql);
//   if(NULL == result) return false;
//   MYSQL_ROW row = mysql_fetch_row(result);

//   if(NULL == row) return false;

//   unsigned int num_fields = mysql_num_fields(result);
//   assert(9 == num_fields);

//   printf("[%s:%d %f] row is: ", __FILE__, __LINE__, now());
//   for(unsigned int i=0; i < num_fields; i++) if(i != 8) printf("%s ", row[i]);
//   printf("\n");

//   RASM::point3d pos(METERS_TO_RASM(atof(row[1])),
// 		    METERS_TO_RASM(atof(row[2])),
// 		    METERS_TO_RASM(atof(row[3])));
//   cloud_pose.set(pos,
// 		 atof(row[4]),  // roll
// 		 atof(row[5]),  // pitch
// 		 atof(row[6]),  // yaw
// 		 atof(row[8])); // time

//   unsigned int sequence_number = atoi(row[0]);

//   printf("[%s:%d %f] cloud %d pose is %f %f %f\n", 
// 	 __FILE__, __LINE__, now(),
// 	 sequence_number,
// 	 RASM_TO_METERS(pos.X()),
// 	 RASM_TO_METERS(pos.Y()),
// 	 cloud_pose.getTime());
//   fflush(stdout);

//   /*
//    * TBD: actually extract points from row[7]
//    */

//   mysql_free_result(result);

//   /*
//    * Now mark this ID as taken
//    */
//   int len = sprintf(query,
// 		    "INSERT INTO mapper_IDs(id) VALUES('%d')",
// 		    sequence_number);
//   assert(len > 0);

//   printf("[%s:%d %f] query returned %d (%d %s)\n", 
// 	 __FILE__, __LINE__, now(),
// 	 mysql_real_query(g_mysql, query, len), errno, strerror(errno));

//   return true;
// }
		     

void send_sql_query(const char* query)
{
  assert(NULL != query);

  /*
   * check to see if we need to connect to the db
   */
  if(NULL == g_mysql) connect_to_db();

  int status = mysql_query(g_mysql, query);

  MYSQL_RES* result = mysql_store_result(g_mysql);

  mysql_free_result(result);
}


#endif // USE_MYSQL
