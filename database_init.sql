-- File: ./database_init.sql (same level as docker-compose.yml)

-- Create databases
CREATE DATABASE barns_oms;
CREATE DATABASE barns_validation;

-- Create users
DO $$
BEGIN
   IF NOT EXISTS (SELECT FROM pg_catalog.pg_roles WHERE rolname = 'barns_user') THEN
      CREATE USER barns_user WITH ENCRYPTED PASSWORD 'barns_pass';
   END IF;
END
$$;

DO $$
BEGIN
   IF NOT EXISTS (SELECT FROM pg_catalog.pg_roles WHERE rolname = 'validation_user') THEN
      CREATE USER validation_user WITH ENCRYPTED PASSWORD 'validation_pass';
   END IF;
END
$$;

-- Grant privileges for OMS
GRANT ALL PRIVILEGES ON DATABASE barns_oms TO barns_user;

-- Grant privileges for Validation  
GRANT ALL PRIVILEGES ON DATABASE barns_validation TO validation_user;

-- Connect to each database and grant schema permissions
\c barns_oms;
GRANT ALL ON SCHEMA public TO barns_user;
GRANT ALL PRIVILEGES ON ALL TABLES IN SCHEMA public TO barns_user;
GRANT ALL PRIVILEGES ON ALL SEQUENCES IN SCHEMA public TO barns_user;
ALTER DEFAULT PRIVILEGES IN SCHEMA public GRANT ALL ON TABLES TO barns_user;
ALTER DEFAULT PRIVILEGES IN SCHEMA public GRANT ALL ON SEQUENCES TO barns_user;

\c barns_validation;
GRANT ALL ON SCHEMA public TO validation_user;
GRANT ALL PRIVILEGES ON ALL TABLES IN SCHEMA public TO validation_user;
GRANT ALL PRIVILEGES ON ALL SEQUENCES IN SCHEMA public TO validation_user;
ALTER DEFAULT PRIVILEGES IN SCHEMA public GRANT ALL ON TABLES TO validation_user;
ALTER DEFAULT PRIVILEGES IN SCHEMA public GRANT ALL ON SEQUENCES TO validation_user;