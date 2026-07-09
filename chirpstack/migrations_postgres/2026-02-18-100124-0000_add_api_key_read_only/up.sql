alter table api_key
  add column is_read_only boolean not null default false;

alter table api_key
  alter column is_read_only drop default;

