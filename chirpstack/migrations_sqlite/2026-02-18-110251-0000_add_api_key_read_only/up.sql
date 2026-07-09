alter table api_key
  add column is_read_only boolean not null default false;

